fn main() {
    let root = gtsam_root();
    let include = root.join("include");
    let library = ["lib", "lib64"]
        .into_iter()
        .map(|name| root.join(name))
        .find(|path| path.is_dir())
        .unwrap_or_else(|| panic!("{} has no lib or lib64 directory", root.display()));
    if !include.join("gtsam/config.h").is_file() {
        panic!(
            "{} does not contain include/gtsam/config.h; run ./scripts/setup-gtsam.sh",
            root.display()
        );
    }

    let mut bridge = cxx_build::bridge("src/lib.rs");
    bridge
        .file("src/gtsam_estimator.cc")
        .include("include")
        .std("c++17")
        .flag(&format!("-isystem{}", include.display()))
        .flag(&format!(
            "-isystem{}",
            include.join("gtsam/3rdparty/Eigen").display()
        ));
    if let Some(boost_root) = boost_root() {
        bridge.flag(&format!("-isystem{}/include", boost_root.display()));
    }
    bridge.compile("fusion-gtsam");

    println!("cargo:rustc-link-search=native={}", library.display());
    println!("cargo:rustc-link-lib=dylib=gtsam");
    println!("cargo:rustc-link-lib=dylib=metis-gtsam");
    if std::env::var("CARGO_CFG_TARGET_FAMILY").as_deref() == Ok("unix") {
        println!("cargo:rustc-link-arg=-Wl,-rpath,{}", library.display());
    }
    println!("cargo:rerun-if-env-changed=GTSAM_ROOT");
    println!("cargo:rerun-if-env-changed=BOOST_ROOT");
    println!("cargo:rerun-if-changed=src/lib.rs");
    println!("cargo:rerun-if-changed=src/gtsam_estimator.cc");
    println!("cargo:rerun-if-changed=include/fusion-gtsam/gtsam_estimator.hpp");
}

fn gtsam_root() -> std::path::PathBuf {
    std::env::var_os("GTSAM_ROOT")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| {
            std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../third_party/gtsam/install")
        })
}

fn boost_root() -> Option<std::path::PathBuf> {
    std::env::var_os("BOOST_ROOT")
        .map(std::path::PathBuf::from)
        .or_else(|| {
            ["/opt/homebrew/opt/boost", "/usr/local/opt/boost"]
                .into_iter()
                .map(std::path::PathBuf::from)
                .find(|path| path.join("include/boost").is_dir())
        })
}
