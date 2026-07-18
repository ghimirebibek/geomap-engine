fn main() {
    // Fixture files (see src/fixture.rs) are JSON encodings of these
    // messages; #[serde(default)] lets a fixture omit fields it doesn't
    // care about instead of having to spell out every one.
    prost_build::Config::new()
        .type_attribute(".", "#[derive(serde::Serialize, serde::Deserialize)]")
        .type_attribute(".", "#[serde(default)]")
        .compile_protos(&["proto/frame.proto"], &["proto/"])
        .unwrap();
}
