mod support;

use std::time::Duration;

#[test]
fn tui_starts_after_confirm_and_exits_on_ctrl_c() {
    if !support::integration_ros_enabled() {
        eprintln!("skipping: set TERMVIZ_IT_ROS=1 to enable ROS integration tests");
        return;
    }
    if !support::command_exists("roscore") {
        eprintln!("skipping: `roscore` not found in PATH");
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

    let env = vec![
        ("ROS_MASTER_URI", ros_master_uri.as_str()),
        ("ROS_IP", "127.0.0.1"),
        ("ROS_LOG_DIR", tmp.path().to_str().unwrap()),
        ("XDG_CONFIG_HOME", tmp.path().to_str().unwrap()),
    ];

    // TF won't be available; with tf-wait-time=0 we quickly reach the confirm prompt.
    let args = vec![cfg_path.as_os_str(), OsStr::new("-t"), OsStr::new("0")];

    let mut run = match support::spawn_termviz_pty(args, &env) {
        Ok(r) => r,
        Err(e) => {
            eprintln!("skipping: failed to start termviz under PTY: {e}");
            return;
        }
    };

    // Wait for the confirm prompt, then answer yes.
    let saw_prompt = run.wait_for_output_contains("Continue", Duration::from_secs(5));
    assert!(saw_prompt, "did not see TF continue prompt; output: {}", run.output.lock().unwrap());

    run.send("y\n");

    let started_tui = run.wait_for_output_contains("Initiating terminal", Duration::from_secs(5));
    assert!(
        started_tui,
        "did not reach terminal init; output: {}",
        run.output.lock().unwrap()
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

use std::ffi::OsStr;
