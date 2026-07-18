use geomap_engine::{replay_fixture, EngineConfig};

#[test]
fn replays_a_recorded_session_and_prunes_the_stale_object() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/session.json");
    let snapshots = replay_fixture(path, EngineConfig::default()).expect("fixture should replay");

    assert_eq!(snapshots.len(), 3);

    // Frame 1: first sighting of the chair.
    assert_eq!(snapshots[0].objects.len(), 1);
    assert_eq!(snapshots[0].objects[0].observation_count, 1);

    // Frame 2 (0.5s later): the same chair is reinforced, not duplicated.
    assert_eq!(snapshots[1].objects.len(), 1);
    assert_eq!(snapshots[1].objects[0].observation_count, 2);
    assert!(snapshots[1].objects[0].confidence > snapshots[0].objects[0].confidence);

    // Frame 3 (4.5s later, no detections, past the 2s stale timeout):
    // the unreinforced chair has aged out of the map.
    assert!(snapshots[2].objects.is_empty());
}
