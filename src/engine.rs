use uuid::Uuid;

use crate::projection::project_to_ground;
use crate::proto::{Detection, Frame, MapObject, SceneMap};

/// Tunable thresholds for association and map maintenance.
#[derive(Debug, Clone, Copy)]
pub struct EngineConfig {
    /// Detections farther than this from an existing object (in meters)
    /// are treated as a new object rather than a repeat observation of it.
    ///
    /// 0.3m comes from the ground-plane projection's noise sensitivity: for
    /// this v0.1 monocular ground-plane method, a fixed pixel error in the
    /// detector's bbox translates to a ground-position error that grows
    /// sharply with distance/viewing angle (near-grazing rays amplify small
    /// pixel noise into large position swings). At the indoor near-to-medium
    /// range this v0.1 engine targets (up to ~4m), +/-8-10px of detector
    /// localization error — representative of a moderate real-time
    /// detector — stays under ~0.26m of ground jitter, so 0.3m has margin
    /// without being so loose that distinct nearby objects merge. Beyond
    /// ~5-8m or at shallow viewing angles this margin erodes fast; that's
    /// an inherent limitation of ground-plane-only projection, not
    /// something this constant can fix. Validated against real camera
    /// motion with synthetic +/-8px detector noise in
    /// tests/replay_tum_fixture.rs / tests/fixtures/tum_freiburg1_xyz.json.
    pub association_radius_meters: f32,
    /// An object not reinforced by a new observation within this many
    /// seconds (frame-timestamp time, not wall-clock) is dropped. One
    /// rule covers all of map maintenance: a one-off false detection
    /// never gets reinforced and ages out; a genuinely stale object ages
    /// out; an object that moved ages out at its old position while a
    /// new one forms at the new position.
    ///
    /// 2.0s is a reasoned assumption, not a measured one: object detection
    /// is usually the expensive stage of a mobile perception pipeline and
    /// commonly throttled well below camera framerate, so assume ~5Hz
    /// detections and tolerate ~10 consecutive misses (occlusion, motion
    /// blur) before treating an object as gone. There's no real detector
    /// dropout data behind this yet — revisit once a live frontend exists.
    pub stale_timeout_seconds: f64,
}

impl Default for EngineConfig {
    fn default() -> Self {
        Self { association_radius_meters: 0.3, stale_timeout_seconds: 2.0 }
    }
}

pub struct Engine {
    scene_map: SceneMap,
    config: EngineConfig,
}

impl Engine {
    pub fn new() -> Self {
        Self::with_config(EngineConfig::default())
    }

    pub fn with_config(config: EngineConfig) -> Self {
        Self { scene_map: SceneMap::default(), config }
    }

    pub fn ingest_frame(&mut self, frame: Frame) -> &SceneMap {
        let (Some(pose), Some(intrinsics)) = (frame.pose.as_ref(), frame.intrinsics.as_ref())
        else {
            return &self.scene_map;
        };

        for detection in &frame.detections {
            if let Some((x, y)) = project_to_ground(pose, intrinsics, detection) {
                self.associate_and_fuse(detection, x, y, pose.timestamp);
            }
        }

        self.prune_stale(pose.timestamp);
        self.scene_map.updated_at = pose.timestamp;
        &self.scene_map
    }

    fn associate_and_fuse(&mut self, detection: &Detection, x: f32, y: f32, timestamp: f64) {
        let radius = self.config.association_radius_meters;
        let nearest = self
            .scene_map
            .objects
            .iter_mut()
            .filter(|obj| obj.label == detection.label)
            .filter_map(|obj| {
                let dist = ((obj.x - x).powi(2) + (obj.y - y).powi(2)).sqrt();
                (dist <= radius).then_some((dist, obj))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, obj)| obj);

        match nearest {
            Some(obj) => fuse(obj, detection, x, y, timestamp),
            None => self.scene_map.objects.push(new_object(detection, x, y, timestamp)),
        }
    }

    fn prune_stale(&mut self, now: f64) {
        let stale_timeout = self.config.stale_timeout_seconds;
        self.scene_map
            .objects
            .retain(|obj| now - obj.last_seen <= stale_timeout);
    }
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}

fn new_object(detection: &Detection, x: f32, y: f32, timestamp: f64) -> MapObject {
    MapObject {
        id: Uuid::new_v4().to_string(),
        label: detection.label.clone(),
        x,
        y,
        confidence: detection.confidence,
        observation_count: 1,
        first_seen: timestamp,
        last_seen: timestamp,
    }
}

fn fuse(obj: &mut MapObject, detection: &Detection, x: f32, y: f32, timestamp: f64) {
    let n = obj.observation_count as f32;
    obj.x = (obj.x * n + x) / (n + 1.0);
    obj.y = (obj.y * n + y) / (n + 1.0);
    // Combine as independent evidence so confidence grows toward 1 with
    // repeated observations, instead of just averaging toward the mean.
    obj.confidence = 1.0 - (1.0 - obj.confidence) * (1.0 - detection.confidence);
    obj.observation_count += 1;
    obj.last_seen = timestamp;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::proto::{CameraIntrinsics, CameraPose};

    fn frame_at(timestamp: f64, x: f32, y: f32, z: f32, detections: Vec<Detection>) -> Frame {
        Frame {
            pose: Some(CameraPose {
                timestamp,
                x,
                y,
                z,
                qx: 1.0,
                qy: 0.0,
                qz: 0.0,
                qw: 0.0,
            }),
            intrinsics: Some(CameraIntrinsics { fx: 500.0, fy: 500.0, cx: 320.0, cy: 240.0 }),
            detections,
        }
    }

    fn detection(label: &str, confidence: f32) -> Detection {
        Detection {
            label: label.to_string(),
            confidence,
            bbox_x: 0.5,
            bbox_y: 0.5,
            bbox_w: 0.0,
            bbox_h: 0.0,
        }
    }

    #[test]
    fn repeated_observations_fuse_into_one_object_with_growing_confidence() {
        let mut engine = Engine::new();
        engine.ingest_frame(frame_at(0.0, 0.0, 0.0, 2.0, vec![detection("chair", 0.5)]));
        let map = engine.ingest_frame(frame_at(0.5, 0.0, 0.0, 2.0, vec![detection("chair", 0.5)]));

        assert_eq!(map.objects.len(), 1);
        let obj = &map.objects[0];
        assert_eq!(obj.observation_count, 2);
        assert!(obj.confidence > 0.5, "confidence should grow past a single observation");
    }

    #[test]
    fn different_labels_at_same_spot_stay_separate() {
        let mut engine = Engine::new();
        let map = engine.ingest_frame(frame_at(
            0.0,
            0.0,
            0.0,
            2.0,
            vec![detection("chair", 0.5), detection("door", 0.5)],
        ));

        assert_eq!(map.objects.len(), 2);
    }

    #[test]
    fn objects_not_reinforced_go_stale_and_get_pruned() {
        let mut engine = Engine::new();
        engine.ingest_frame(frame_at(0.0, 0.0, 0.0, 2.0, vec![detection("chair", 0.5)]));
        let map = engine.ingest_frame(frame_at(10.0, 0.0, 0.0, 2.0, vec![]));

        assert!(map.objects.is_empty());
    }
}
