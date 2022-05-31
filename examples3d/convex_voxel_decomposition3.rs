use obj::raw::object::Polygon;
use rapier3d::parry::bounding_volume;
use rapier3d::parry::transformation::voxelization::FillMode;
use rapier3d::prelude::*;
use rapier_testbed3d::Testbed;
use std::fs::File;
use std::io::BufReader;

const VOXEL_SIZE: Real = 0.25;
/*
 * NOTE: The `r` macro is only here to convert from f64 to the `N` scalar type.
 * This simplifies experimentation with various scalar types (f32, fixed-point numbers, etc.)
 */
pub fn init_world(testbed: &mut Testbed) {
    /*
     * World
     */
    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let impulse_joints = ImpulseJointSet::new();
    let multibody_joints = MultibodyJointSet::new();

    /*
     * Create a bowl for the ground
     */
    {
        let obj_path = "assets/3d/bowl.obj";
        let input = BufReader::new(File::open(obj_path).unwrap());

        if let Ok(model) = obj::raw::parse_obj(input) {
            let mut vertices: Vec<_> = model
                .positions
                .iter()
                .map(|v| point![v.0, v.1, v.2])
                .collect();
            let indices: Vec<_> = model
                .polygons
                .into_iter()
                .flat_map(|p| match p {
                    Polygon::P(idx) => idx.into_iter(),
                    Polygon::PT(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
                    Polygon::PN(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
                    Polygon::PTN(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
                })
                .collect();

            // Compute the size of the model, to scale it and have similar size for everything.
            let aabb = bounding_volume::details::local_point_cloud_aabb(&vertices);
            let center = aabb.center();
            let diag = (aabb.maxs - aabb.mins).norm();

            vertices
                .iter_mut()
                .for_each(|p| *p = (*p - center.coords) * 0.2);

            let indices: Vec<_> = indices
                .chunks(3)
                .map(|idx| [idx[0] as u32, idx[1] as u32, idx[2] as u32])
                .collect();

            let voxels =
                SharedShape::voxelized_mesh(&vertices, &indices, VOXEL_SIZE, FillMode::default());
            let collider = ColliderBuilder::new(voxels);
            colliders.insert(collider);
        }
    }

    /*
     * Create the convex decompositions.
     */
    let geoms = models();
    let ngeoms = geoms.len();
    let width = (ngeoms as f32).sqrt() as usize;
    let num_duplications = 1; // 4;
    let shift = 7.0f32;

    for (igeom, obj_path) in geoms.into_iter().enumerate() {
        let deltas = Isometry::identity();

        let mut shapes = Vec::new();
        println!("Parsing and decomposing: {}", obj_path);

        let input = BufReader::new(File::open(obj_path).unwrap());

        if let Ok(model) = obj::raw::parse_obj(input) {
            let mut vertices: Vec<_> = model
                .positions
                .iter()
                .map(|v| point![v.0, v.1, v.2])
                .collect();
            let indices: Vec<_> = model
                .polygons
                .into_iter()
                .flat_map(|p| match p {
                    Polygon::P(idx) => idx.into_iter(),
                    Polygon::PT(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
                    Polygon::PN(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
                    Polygon::PTN(idx) => Vec::from_iter(idx.into_iter().map(|i| i.0)).into_iter(),
                })
                .collect();

            // Compute the size of the model, to scale it and have similar size for everything.
            let aabb = bounding_volume::details::point_cloud_aabb(&deltas, &vertices);
            let center = aabb.center();
            let diag = (aabb.maxs - aabb.mins).norm();

            vertices
                .iter_mut()
                .for_each(|p| *p = (*p - center.coords) * 6.0 / diag);

            let indices: Vec<_> = indices
                .chunks(3)
                .map(|idx| [idx[0] as u32, idx[1] as u32, idx[2] as u32])
                .collect();

            let decomposed_shape =
                SharedShape::voxelized_mesh(&vertices, &indices, VOXEL_SIZE, FillMode::default());
            shapes.push(decomposed_shape);

            for k in 1..num_duplications + 1 {
                let x = (igeom % width) as f32 * shift - 3.0;
                let y = (igeom / width) as f32 * shift + 4.0;
                let z = k as f32 * shift - 3.0;

                let body = RigidBodyBuilder::dynamic().translation(vector![x, y, z]);
                let handle = bodies.insert(body);

                for shape in &shapes {
                    let collider = ColliderBuilder::new(shape.clone());
                    colliders.insert_with_parent(collider, handle, &mut bodies);
                }
            }
        }
    }

    /*
     * Set up the testbed.
     */
    testbed.set_world(bodies, colliders, impulse_joints, multibody_joints);
    testbed.look_at(point![100.0, 100.0, 100.0], Point::origin());
}

fn models() -> Vec<String> {
    vec![
        // "assets/3d/camel_decimated.obj".to_string(),
        "assets/3d/chair.obj".to_string(),
        "assets/3d/cup_decimated.obj".to_string(),
        "assets/3d/dilo_decimated.obj".to_string(),
        "assets/3d/feline_decimated.obj".to_string(),
        "assets/3d/genus3_decimated.obj".to_string(),
        "assets/3d/hand2_decimated.obj".to_string(),
        "assets/3d/hand_decimated.obj".to_string(),
        "assets/3d/hornbug.obj".to_string(),
        "assets/3d/octopus_decimated.obj".to_string(),
        "assets/3d/rabbit_decimated.obj".to_string(),
        // "assets/3d/rust_logo.obj".to_string(),
        "assets/3d/rust_logo_simplified.obj".to_string(),
        "assets/3d/screwdriver_decimated.obj".to_string(),
        "assets/3d/table.obj".to_string(),
        "assets/3d/tstTorusModel.obj".to_string(),
        // "assets/3d/tstTorusModel2.obj".to_string(),
        // "assets/3d/tstTorusModel3.obj".to_string(),
    ]
}
