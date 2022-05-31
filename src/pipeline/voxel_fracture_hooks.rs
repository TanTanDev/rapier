use crate::dynamics::RigidBodyBuilder;
use crate::geometry::{Collider, ColliderBuilder, ColliderHandle, ContactPair, AABB};
use crate::math::{Point, Real, Vector};
use crate::pipeline::{EventHandler, PhysicsHooks, PostSolveContext};
use parry::{shape::SharedShape, utils::hashmap::HashMap};

#[derive(Copy, Clone, PartialEq, Debug)]
pub struct VoxelFractureMaterial {
    pub max_momentum: Real,
    pub min_force: Real,
    pub normal_dissipation: Real,
    pub max_fracture_radius: Real,
}

impl Default for VoxelFractureMaterial {
    fn default() -> Self {
        Self {
            max_momentum: 10.0,
            min_force: 1.0,
            normal_dissipation: 1.0,
            max_fracture_radius: 1.0,
        }
    }
}

pub struct FractureEvent {
    pub fractured_collider: ColliderHandle,
    pub fragments: Vec<ColliderHandle>,
}

#[derive(Clone, Debug, Default)]
pub struct VoxelFractureHooks {
    pub default_material: VoxelFractureMaterial,
    // TODO: allow one material per voxel collider.
}

impl VoxelFractureHooks {
    pub fn post_solve(&self, context: &mut PostSolveContext, events: &dyn EventHandler) {
        let t0 = instant::now();
        let mut fragments: HashMap<ColliderHandle, Vec<ColliderHandle>> = Default::default();

        let mut num_pairs = 0;
        let mut num_fract = 0;

        for active_pair in context
            .active_pairs
            .iter()
            .map(|id| context.narrow_phase.contact_pair_at_index(*id))
        {
            num_pairs += 1;
            let co1 = &context.colliders[active_pair.collider1];
            let co2 = &context.colliders[active_pair.collider2];

            if co1.shape.as_voxels().is_none() && co2.shape.as_voxels().is_none() {
                continue;
            }

            for manifold in &active_pair.manifolds {
                if let Some(point) = &manifold.find_deepest_contact() {
                    // Compute the fracture pattern at this contact.
                    // momentum = force * dist => dist = momentum / force
                    let force = point.data.impulse * context.integration_parameters.inv_dt();

                    if force < self.default_material.min_force {
                        continue;
                    }

                    let inv_force = crate::utils::inv(force);
                    let fracture_radius = self.default_material.max_momentum * inv_force;

                    if fracture_radius < self.default_material.max_fracture_radius {
                        num_fract += 1;
                        // Fracture the voxel model.
                        let mut fracture_collider =
                            |handle: ColliderHandle,
                             local_n: Vector<Real>,
                             local_p: Point<Real>| {
                                let mut collider = &mut context.colliders[handle];

                                if let Some(vox) = collider.shape.as_voxels() {
                                    let fracture_normal = local_n;
                                    let fracture_depth = fracture_radius; // TODO: find a good force-dependant depth.
                                    let fracture_dir_id = fracture_normal.iamax();
                                    let tangent_a = Vector::ith((fracture_dir_id + 1) % 3, 1.0);
                                    let tangent_b = Vector::ith((fracture_dir_id + 2) % 3, 1.0);
                                    let mut fracture_box = AABB {
                                        mins: local_p - (tangent_a + tangent_b) * fracture_radius,
                                        maxs: local_p + (tangent_a + tangent_b) * fracture_radius,
                                    };

                                    if fracture_normal[fracture_dir_id] > 0.0 {
                                        fracture_box.mins[fracture_dir_id] -= fracture_depth;
                                    } else {
                                        fracture_box.maxs[fracture_dir_id] += fracture_depth;
                                    }

                                    if let (Some(in_box), Some(rest)) =
                                        vox.split_with_box(&fracture_box)
                                    {
                                        *collider.shape_mut().as_voxels_mut().unwrap() = rest;
                                        // Create a new, dynamic body with the detached part.
                                        let body = RigidBodyBuilder::dynamic()
                                            .position(*collider.position());
                                        let collider =
                                            ColliderBuilder::new(SharedShape::new(in_box));
                                        let body_handle = context.bodies.insert(body);
                                        let fragment_handle = context.colliders.insert_with_parent(
                                            collider,
                                            body_handle,
                                            context.bodies,
                                        );

                                        fragments
                                            .entry(handle)
                                            .or_insert_with(|| vec![])
                                            .push(fragment_handle);
                                    }
                                }
                            };

                        fracture_collider(active_pair.collider1, manifold.local_n1, point.local_p1);
                        fracture_collider(active_pair.collider2, manifold.local_n2, point.local_p2);
                    }
                }
            }
        }

        for (handle, fragments) in fragments.drain() {
            let event = FractureEvent {
                fractured_collider: handle,
                fragments,
            };
            events.handle_fracture_event(context.bodies, context.colliders, event);
        }

        // let t1 = instant::now();
        // dbg!(t1 - t0);
        // dbg!(num_pairs, num_fract);
    }
}
