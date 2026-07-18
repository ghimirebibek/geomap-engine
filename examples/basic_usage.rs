// Run with: cargo run --example basic_usage
use geomap_engine::Engine;
use geomap_engine::proto::{CameraIntrinsics, CameraPose, Detection, Frame};

fn chair_frame(timestamp: f64) -> Frame {
    Frame {
        pose: Some(CameraPose {
            timestamp,
            x: 0.0, y: 0.0, z: 1.5, // 1.5m camera height above the floor
            qx: 1.0, qy: 0.0, qz: 0.0, qw: 0.0, // looking straight down
        }),
        intrinsics: Some(CameraIntrinsics { fx: 500.0, fy: 500.0, cx: 320.0, cy: 240.0 }),
        detections: vec![Detection {
            label: "chair".into(),
            confidence: 0.8,
            bbox_x: 0.45, bbox_y: 0.45, bbox_w: 0.1, bbox_h: 0.1,
        }],
    }
}

fn main() {
    let mut engine = Engine::new();

    // Two sightings of the same chair, 0.5s apart, fuse into one
    // MapObject instead of piling up as two separate detections.
    engine.ingest_frame(chair_frame(0.0));
    let scene_map = engine.ingest_frame(chair_frame(0.5));

    let chair = &scene_map.objects[0];
    println!(
        "fused '{}' at ({:.2}, {:.2}), confidence {:.2} from {} observation(s)",
        chair.label, chair.x, chair.y, chair.confidence, chair.observation_count
    );
    println!("\nfull SceneMap as GeoJSON:\n{}", scene_map.to_geojson_json().unwrap());
}
