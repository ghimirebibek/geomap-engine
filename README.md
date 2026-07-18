# geomap-engine

**A Rust engine that fuses camera pose (ARKit, ARCore, or any SLAM
source) with object detections into a persistent, deduplicated 2D map of
objects in a physical space.**

[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
[![Rust edition 2021](https://img.shields.io/badge/edition-2021-orange.svg)](Cargo.toml)

Feed it a stream of `{camera pose, object detections}` per frame; it
maintains a clean, queryable `SceneMap` of the objects in the space —
one entry per real-world object, not one per detection. Any mobile app,
robot, or spatial-computing tool can sit on top of it.

## What this is (and isn't)

Existing tools (RoomPlan, Polycam, ARCore/ARKit) bundle SLAM + object
detection + UI into closed or heavyweight systems. There's no small,
embeddable, open-source piece that just does pose+detection fusion well.
geomap-engine fills that specific gap.

It is **not** a full AR app, and does not do any of the following itself:
- **No SLAM** — it consumes camera pose, it doesn't produce it (bring
  your own ARKit/ARCore/ORB-SLAM3/etc.)
- **No object detection model** — it consumes detections (label +
  bounding box), it doesn't run inference
- **No mobile UI or app shell**
- **No depth/LiDAR requirement** — works with monocular pose only

## Status

v0.1 core is implemented and tested (11 tests, `cargo build` / `cargo
test` / `cargo clippy` all clean): ground-plane projection, nearest-
neighbor association, running-average fusion with growing confidence,
and staleness-based map maintenance. Pre-1.0 — the public API may still
change. See [Roadmap](#roadmap-post-v01) for what's intentionally not
built yet.

## Quick start

### Prerequisites
- Rust (stable, edition 2021+)
- [`protoc`](https://github.com/protocolbuffers/protobuf) — the
  Protocol Buffers compiler, used by `build.rs` to generate types from
  [proto/frame.proto](proto/frame.proto). Install it with:
  - macOS: `brew install protobuf`
  - Debian/Ubuntu: `apt install protobuf-compiler`
  - or download a release directly from the
    [protobuf releases page](https://github.com/protocolbuffers/protobuf/releases)

### Build from source
```sh
git clone https://github.com/ghimirebibek/geomap-engine
cd geomap-engine
cargo build
cargo test
```

### Use as a dependency
Not published to crates.io yet — pull it straight from GitHub:
```toml
[dependencies]
geomap-engine = { git = "https://github.com/ghimirebibek/geomap-engine" }
```

### Example
```rust
use geomap_engine::Engine;
use geomap_engine::proto::{CameraIntrinsics, CameraPose, Detection, Frame};

let mut engine = Engine::new();

let frame = Frame {
    pose: Some(CameraPose {
        timestamp: 0.0,
        x: 0.0, y: 0.0, z: 1.5, // 1.5m camera height above the floor
        qx: 1.0, qy: 0.0, qz: 0.0, qw: 0.0, // looking straight down
    }),
    intrinsics: Some(CameraIntrinsics { fx: 500.0, fy: 500.0, cx: 320.0, cy: 240.0 }),
    detections: vec![Detection {
        label: "chair".into(),
        confidence: 0.8,
        bbox_x: 0.45, bbox_y: 0.45, bbox_w: 0.1, bbox_h: 0.1,
    }],
};

let scene_map = engine.ingest_frame(frame);
println!("{}", scene_map.to_geojson_json().unwrap());
```

Feed it more frames over time and `SceneMap` converges to one stable,
deduplicated entry per real object — repeated sightings of the same
chair fuse into one `MapObject` with growing confidence, instead of
piling up as separate detections. See
[examples/basic_usage.rs](examples/basic_usage.rs) (`cargo run --example
basic_usage`) for a runnable version that shows this fusion happening
across two frames.

## How it works

1. **Projection** — turns a 2D bounding box + camera pose + intrinsics
   into an estimated 2D world position, via ray-plane intersection with
   the ground plane (`z = 0`), using the pose's height above ground
   (ground-plane assumption, v0.1). See [src/projection.rs](src/projection.rs).
2. **Association** — decides whether a new detection matches an existing
   tracked object, using label match + position proximity (naive
   nearest-neighbor within a configurable radius).
3. **Fusion** — merges repeated observations into one stable object:
   running-average position, confidence combined as independent evidence
   so it grows toward 1 with repeated sightings.
4. **Map maintenance** — a single staleness rule (no reinforcement within
   a configurable timeout) handles pruning one-off noise, stale objects,
   and objects that moved, all at once.

Association radius and stale timeout are both tunable via `EngineConfig`
(see [src/engine.rs](src/engine.rs)) — the default association radius is
derived from an actual noise-sensitivity analysis of the ground-plane
projection, documented in the code, not picked arbitrarily.

## Tech stack
- **Language:** Rust — for portability and future FFI into iOS/Android
- **Schema:** Protobuf (see [proto/frame.proto](proto/frame.proto)) for
  input `Frame` and output `SceneMap`, so any front-end (Swift/Kotlin/
  test harness) can feed it language-agnostically
- **Public API surface:** minimal — `Engine::ingest_frame(Frame) ->
  &SceneMap`, plus `SceneMap::to_geojson()` for output

## v0.1 scope
- Fixed input: stream of `{timestamp, camera_pose, detections[]}`
- Ground-plane-assumption projection (known/fixed camera height)
- Naive nearest-neighbor association + running-average fusion
- Output: `SceneMap` as a GeoJSON-like structure (`FeatureCollection` of
  `Point` features), testable via matplotlib/geopandas without any
  mobile app involved

## Testing strategy
No phone required for v0.1 development. `replay_fixture()` (see
[src/fixture.rs](src/fixture.rs)) loads a JSON array of `Frame` messages
and replays them through an `Engine`, returning a `SceneMap` snapshot
per frame — recorded pose+detection logs as replayable input fixtures,
instead of a live SLAM/detector source.

- [tests/fixtures/session.json](tests/fixtures/session.json) — a small,
  hand-written fixture exercising fusion and staleness pruning
  ([tests/replay.rs](tests/replay.rs)).
- [tests/fixtures/tum_freiburg1_xyz.json](tests/fixtures/tum_freiburg1_xyz.json) —
  pairs a **real** camera trajectory from the [TUM RGB-D SLAM Dataset and
  Benchmark](https://cvg.cit.tum.de/data/datasets/rgbd-dataset) (Sturm et
  al., Freiburg1/xyz sequence, CC BY 4.0) with synthetic detections
  (including deterministic detector-noise jitter), generated by
  [scripts/gen_tum_fixture.py](scripts/gen_tum_fixture.py). TUM RGB-D has
  no object-detection ground truth, so only the pose stream is real; see
  the script's docstring for how detections were derived and its
  caveats. Exercised by [tests/replay_tum_fixture.rs](tests/replay_tum_fixture.rs),
  which also proves the association radius is doing real work (a too-tight
  radius visibly fragments objects under realistic noise; the default
  doesn't).

## Contributing

Pull requests welcome. Before submitting:
```sh
cargo build
cargo test
cargo clippy --all-targets
cargo fmt
```
Keep changes scoped — this is a small, deliberately minimal v0.1 engine
(see [Non-goals](#what-this-is-and-isnt) and [Roadmap](#roadmap-post-v01)
before proposing anything that expands scope). Bug fixes, test coverage,
and documentation improvements are always welcome; for larger features
or API changes, open an issue first to discuss direction.

## Roadmap (post-v0.1)
- Depth-aware projection when available
- Visual re-identification for association (not just position/class)
- Moving object handling
- Multi-session map merging
- CI (build/test/clippy on every push and PR)

## License
[MIT](LICENSE)
