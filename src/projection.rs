use crate::proto::{CameraIntrinsics, CameraPose, Detection};

/// Detection bboxes are normalized [0,1], but CameraIntrinsics are in
/// pixels, and Frame carries no explicit image width/height to convert
/// between the two. We assume the principal point is the image center
/// (a standard pinhole simplification), which gives width = 2*cx,
/// height = 2*cy and lets normalized coordinates be scaled to pixels.
fn pixel_from_normalized(norm: f32, principal: f32) -> f32 {
    norm * 2.0 * principal
}

/// Projects a detection's bbox center onto the ground plane (world z = 0)
/// by casting a ray through the pinhole camera model and intersecting it
/// with the ground, using the pose's height above ground (pose.z).
/// Returns None if the ray can't hit the ground (level/upward look, or
/// the intersection falls behind the camera).
pub fn project_to_ground(
    pose: &CameraPose,
    intrinsics: &CameraIntrinsics,
    detection: &Detection,
) -> Option<(f32, f32)> {
    let cx_norm = detection.bbox_x + detection.bbox_w / 2.0;
    let cy_norm = detection.bbox_y + detection.bbox_h / 2.0;

    let px = pixel_from_normalized(cx_norm, intrinsics.cx);
    let py = pixel_from_normalized(cy_norm, intrinsics.cy);

    let dir_cam = (
        (px - intrinsics.cx) / intrinsics.fx,
        (py - intrinsics.cy) / intrinsics.fy,
        1.0_f32,
    );

    let dir_world = rotate_by_quat(dir_cam, (pose.qx, pose.qy, pose.qz, pose.qw));

    if dir_world.2 >= -f32::EPSILON {
        return None;
    }

    let t = -pose.z / dir_world.2;
    if t <= 0.0 {
        return None;
    }

    Some((pose.x + t * dir_world.0, pose.y + t * dir_world.1))
}

type Vec3 = (f32, f32, f32);

fn rotate_by_quat(v: Vec3, q: (f32, f32, f32, f32)) -> Vec3 {
    let (qx, qy, qz, qw) = q;
    let qv = (qx, qy, qz);

    let cross1 = cross(qv, v);
    let cross2 = cross(qv, cross1);

    add(add(v, scale(cross1, 2.0 * qw)), scale(cross2, 2.0))
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    (
        a.1 * b.2 - a.2 * b.1,
        a.2 * b.0 - a.0 * b.2,
        a.0 * b.1 - a.1 * b.0,
    )
}

fn scale(v: Vec3, s: f32) -> Vec3 {
    (v.0 * s, v.1 * s, v.2 * s)
}

fn add(a: Vec3, b: Vec3) -> Vec3 {
    (a.0 + b.0, a.1 + b.1, a.2 + b.2)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn intrinsics() -> CameraIntrinsics {
        CameraIntrinsics { fx: 500.0, fy: 500.0, cx: 320.0, cy: 240.0 }
    }

    fn detection_at_center() -> Detection {
        Detection {
            label: "chair".to_string(),
            confidence: 0.9,
            bbox_x: 0.5,
            bbox_y: 0.5,
            bbox_w: 0.0,
            bbox_h: 0.0,
        }
    }

    #[test]
    fn straight_down_camera_projects_directly_below() {
        // 180-degree rotation about x: camera's forward (+z) points at world -z.
        let pose = CameraPose {
            timestamp: 0.0,
            x: 2.0,
            y: 3.0,
            z: 5.0,
            qx: 1.0,
            qy: 0.0,
            qz: 0.0,
            qw: 0.0,
        };

        let (x, y) = project_to_ground(&pose, &intrinsics(), &detection_at_center()).unwrap();
        assert!((x - 2.0).abs() < 1e-4);
        assert!((y - 3.0).abs() < 1e-4);
    }

    #[test]
    fn upward_camera_never_hits_ground() {
        // Identity orientation: forward (+z_cam) points along world +z, away from the ground.
        let pose = CameraPose {
            timestamp: 0.0,
            x: 0.0,
            y: 0.0,
            z: 1.5,
            qx: 0.0,
            qy: 0.0,
            qz: 0.0,
            qw: 1.0,
        };

        assert!(project_to_ground(&pose, &intrinsics(), &detection_at_center()).is_none());
    }
}
