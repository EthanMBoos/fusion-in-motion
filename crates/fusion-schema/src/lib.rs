//! Generated, language-neutral experiment messages.

pub mod messages {
    include!(concat!(env!("OUT_DIR"), "/fusion.rs"));
}

/// Complete transitive descriptor set embedded in every MCAP schema record.
pub const FILE_DESCRIPTOR_SET: &[u8] =
    include_bytes!(concat!(env!("OUT_DIR"), "/fusion_descriptor.bin"));
