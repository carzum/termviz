mod support;

use std::ffi::OsStr;
use std::time::Duration;

#[test]
fn starts_without_tf_prompt_when_static_tf_published() {
    if !support::integration_ros_enabled() {
        eprintln!("skipping: set TERMVIZ_IT_ROS=1 to enable ROS integration tests");
        return;
    }
    if !support::command_exists("roscore") || !support::command_exists("rosrun") {
        eprintln!("skipping: ROS tools not found in PATH (need roscore + rosrun)");
        return;
    }

    let tmp = tempfile::tempdir().unwrap();
    let cfg_path = tmp.path().join("termviz.yml");
    std::fs::write(&cfg_path, support::minimal_config_yaml()).unwrap();

    let ros_port = support::pick_unused_port();
    let ros_master_uri = format!("http://127.0.0.1:{ros_port}");

    let _roscore = match support::spawn_roscore(ros_port, tmp.path()) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("skipping: failed to start roscore: {e}");
            return;
        }
    };

    let _tf = match support::spawn_static_tf(
        ros_port,
        &ros_master_uri,
        "map",
        "base_link",
        tmp.path(),
    ) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("skipping: failed to start static TF publisher: {e}");
            return;
        }
    };

    let env = vec![
        ("ROS_MASTER_URI", ros_master_uri.as_str()),
        ("ROS_IP", "127.0.0.1"),
        ("ROS_LOG_DIR", tmp.path().to_str().unwrap()),
        ("XDG_CONFIG_HOME", tmp.path().to_str().unwrap()),
    ];

    let args = vec![
        cfg_path.as_os_str(),
        OsStr::new("--tf-wait-time"),
        OsStr::new("3"),
    ];

    let mut run = match support::spawn_termviz_pty(args, &env) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("skipping: failed to start termviz under PTY: {e}");
            return;
        }
    };

    let started_tui = run.wait_for_output_contains("Initiating terminal", Duration::from_secs(6));
    assert!(
        started_tui,
        "did not reach terminal init; output: {}",
        run.output.lock().unwrap()
    );

    let output = run.output.lock().unwrap().clone();
    assert!(
        !output.contains("Continue"),
        "did not expect a TF continue prompt; output: {}",
        output
    );

    run.send_ctrl_c();

    let status = run
        .wait_with_timeout(Duration::from_secs(5))
        .unwrap_or_else(|| {
            run.kill();
            panic!("termviz did not exit after Ctrl+C; output: {}", run.output.lock().unwrap());
        });

    assert!(status.success(), "expected success exit; got {:?}", status);
}
