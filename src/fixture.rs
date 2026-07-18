use std::fmt;
use std::fs;
use std::path::Path;

use crate::engine::{Engine, EngineConfig};
use crate::proto::{Frame, SceneMap};

#[derive(Debug)]
pub enum FixtureError {
    Io(std::io::Error),
    Json(serde_json::Error),
}

impl fmt::Display for FixtureError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            FixtureError::Io(e) => write!(f, "failed to read fixture file: {e}"),
            FixtureError::Json(e) => write!(f, "failed to parse fixture JSON: {e}"),
        }
    }
}

impl std::error::Error for FixtureError {}

impl From<std::io::Error> for FixtureError {
    fn from(e: std::io::Error) -> Self {
        FixtureError::Io(e)
    }
}

impl From<serde_json::Error> for FixtureError {
    fn from(e: serde_json::Error) -> Self {
        FixtureError::Json(e)
    }
}

/// Loads a JSON array of Frame messages from `path` and replays them, in
/// order, through a fresh Engine — returning the SceneMap snapshot after
/// each frame. This is the harness the README's testing strategy calls
/// for: recorded pose+detection logs as replayable input fixtures, no
/// phone or live SLAM/detector needed. Snapshots (not just the final map)
/// are returned so the map's evolution over a session can be inspected or
/// plotted.
pub fn replay_fixture(
    path: impl AsRef<Path>,
    config: EngineConfig,
) -> Result<Vec<SceneMap>, FixtureError> {
    let data = fs::read_to_string(path)?;
    let frames: Vec<Frame> = serde_json::from_str(&data)?;

    let mut engine = Engine::with_config(config);
    Ok(frames.into_iter().map(|frame| engine.ingest_frame(frame).clone()).collect())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_file_reports_an_io_error() {
        let result = replay_fixture("/no/such/fixture.json", EngineConfig::default());
        assert!(matches!(result, Err(FixtureError::Io(_))));
    }
}
