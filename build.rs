fn main() {
    prost_build::compile_protos(&["proto/frame.proto"], &["proto/"]).unwrap();
}
