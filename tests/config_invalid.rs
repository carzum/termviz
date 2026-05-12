#[test]
fn invalid_config_fails_before_ros_connect() {
    let dir = tempfile::tempdir().unwrap();
    let cfg_path = dir.path().join("bad.yml");
    std::fs::write(&cfg_path, "this: [is: not: valid").unwrap();

    let mut cmd = assert_cmd::cargo::cargo_bin_cmd!("termviz");
    cmd.arg(cfg_path).assert().failure();
}
