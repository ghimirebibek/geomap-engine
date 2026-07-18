use geomap_engine::{replay_fixture, EngineConfig};

/// The floor-plane positions scripts/gen_tum_fixture.py placed the two
/// synthetic objects at, for comparison against what the engine fuses
/// them into after seeing them through 60 frames of real, noisy camera
/// motion plus +/-8px synthetic detector noise (see
/// tests/fixtures/tum_freiburg1_xyz.json).
const CHAIR_TRUE_POS: (f32, f32) = (-0.26525, 1.1161);
const DOOR_TRUE_POS: (f32, f32) = (-1.26525, 1.1161);
const POSITION_TOLERANCE_METERS: f32 = 0.05;

fn find<'a>(objects: &'a [geomap_engine::proto::MapObject], label: &str) -> &'a geomap_engine::proto::MapObject {
    objects.iter().find(|o| o.label == label).unwrap_or_else(|| panic!("no {label} object in final map"))
}

#[test]
fn real_camera_trajectory_fuses_into_two_stable_objects() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/tum_freiburg1_xyz.json");
    let snapshots = replay_fixture(path, EngineConfig::default()).expect("fixture should replay");

    assert_eq!(snapshots.len(), 60);

    let last = snapshots.last().unwrap();
    // Real trajectory jitter didn't fragment either object into spurious
    // duplicates: exactly the two objects the fixture generator placed.
    assert_eq!(last.objects.len(), 2);

    let chair = find(&last.objects, "chair");
    assert!((chair.x - CHAIR_TRUE_POS.0).abs() < POSITION_TOLERANCE_METERS);
    assert!((chair.y - CHAIR_TRUE_POS.1).abs() < POSITION_TOLERANCE_METERS);
    assert_eq!(chair.observation_count, 60);

    let door = find(&last.objects, "door");
    assert!((door.x - DOOR_TRUE_POS.0).abs() < POSITION_TOLERANCE_METERS);
    assert!((door.y - DOOR_TRUE_POS.1).abs() < POSITION_TOLERANCE_METERS);
    assert_eq!(door.observation_count, 59);

    // Repeated real observations should drive confidence up toward 1.
    assert!(chair.confidence > 0.99);
    assert!(door.confidence > 0.99);
}

/// EngineConfig::default()'s association radius was chosen to comfortably
/// cover this fixture's detector-noise-induced jitter (see src/engine.rs).
/// A radius far below that noise floor should fail to keep up: this
/// pins down that the noise in the fixture is doing real work, not just
/// passing regardless of the radius.
#[test]
fn too_tight_a_radius_fragments_under_realistic_detector_noise() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/tum_freiburg1_xyz.json");
    let config = EngineConfig { association_radius_meters: 0.02, ..EngineConfig::default() };
    let snapshots = replay_fixture(path, config).unwrap();

    let last = snapshots.last().unwrap();
    assert!(
        last.objects.len() > 2,
        "expected a too-tight radius to fragment the two true objects into duplicates, got {}",
        last.objects.len()
    );
}
