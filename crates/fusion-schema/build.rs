use std::{env, path::PathBuf};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let descriptor_path = PathBuf::from(env::var("OUT_DIR")?).join("fusion_descriptor.bin");
    let mut config = prost_build::Config::new();
    config.file_descriptor_set_path(descriptor_path);
    config.compile_protos(&["../../proto/fusion.proto"], &["../../proto"])?;
    println!("cargo:rerun-if-changed=../../proto/fusion.proto");
    Ok(())
}
