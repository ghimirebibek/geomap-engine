pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/geomap.rs"));
}

use proto::{Frame, SceneMap};

pub struct Engine {
    scene_map: SceneMap,
}

impl Engine {
    pub fn new() -> Self {
        Self {
            scene_map: SceneMap::default(),
        }
    }

    pub fn ingest_frame(&mut self, _frame: Frame) -> &SceneMap {
        // TODO: projection, association, fusion, map maintenance
        &self.scene_map
    }
}

impl Default for Engine {
    fn default() -> Self {
        Self::new()
    }
}
