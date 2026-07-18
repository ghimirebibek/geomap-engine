pub mod proto {
    include!(concat!(env!("OUT_DIR"), "/geomap.rs"));
}

mod engine;
mod geojson;
mod projection;

pub use engine::Engine;
pub use geojson::FeatureCollection;
