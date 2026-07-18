use serde::Serialize;

use crate::proto::{MapObject, SceneMap};

/// A GeoJSON-like FeatureCollection: each MapObject becomes a Point
/// feature, so the map can be written to a .geojson file and inspected
/// with matplotlib/geopandas without any mobile app in the loop.
#[derive(Serialize)]
pub struct FeatureCollection {
    #[serde(rename = "type")]
    kind: &'static str,
    updated_at: f64,
    features: Vec<Feature>,
}

#[derive(Serialize)]
struct Feature {
    #[serde(rename = "type")]
    kind: &'static str,
    geometry: Geometry,
    properties: Properties,
}

#[derive(Serialize)]
struct Geometry {
    #[serde(rename = "type")]
    kind: &'static str,
    coordinates: [f32; 2],
}

#[derive(Serialize)]
struct Properties {
    id: String,
    label: String,
    confidence: f32,
    observation_count: i32,
    first_seen: f64,
    last_seen: f64,
}

impl SceneMap {
    pub fn to_geojson(&self) -> FeatureCollection {
        FeatureCollection {
            kind: "FeatureCollection",
            updated_at: self.updated_at,
            features: self.objects.iter().map(Feature::from).collect(),
        }
    }

    pub fn to_geojson_json(&self) -> serde_json::Result<String> {
        serde_json::to_string_pretty(&self.to_geojson())
    }
}

impl From<&MapObject> for Feature {
    fn from(obj: &MapObject) -> Self {
        Feature {
            kind: "Feature",
            geometry: Geometry { kind: "Point", coordinates: [obj.x, obj.y] },
            properties: Properties {
                id: obj.id.clone(),
                label: obj.label.clone(),
                confidence: obj.confidence,
                observation_count: obj.observation_count,
                first_seen: obj.first_seen,
                last_seen: obj.last_seen,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_map() -> SceneMap {
        SceneMap {
            objects: vec![MapObject {
                id: "abc-123".to_string(),
                label: "chair".to_string(),
                x: 1.5,
                y: -2.5,
                confidence: 0.87,
                observation_count: 3,
                first_seen: 10.0,
                last_seen: 12.5,
            }],
            updated_at: 12.5,
        }
    }

    #[test]
    fn converts_map_objects_to_point_features() {
        let fc = sample_map().to_geojson();
        assert_eq!(fc.features.len(), 1);
        assert_eq!(fc.features[0].geometry.coordinates, [1.5, -2.5]);
        assert_eq!(fc.features[0].properties.label, "chair");
    }

    #[test]
    fn serializes_to_valid_geojson_shaped_json() {
        let json = sample_map().to_geojson_json().unwrap();
        let value: serde_json::Value = serde_json::from_str(&json).unwrap();

        assert_eq!(value["type"], "FeatureCollection");
        assert_eq!(value["features"][0]["type"], "Feature");
        assert_eq!(value["features"][0]["geometry"]["type"], "Point");
        assert_eq!(value["features"][0]["geometry"]["coordinates"][0], 1.5);
        assert_eq!(value["features"][0]["properties"]["id"], "abc-123");
    }
}
