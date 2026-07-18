pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/geomap.rs"));
}

mod engine;
mod fixture;
mod geojson;
mod projection;

pub use engine::{Engine, EngineConfig};
pub use fixture::{replay_fixture, FixtureError};
pub use geojson::FeatureCollection;
