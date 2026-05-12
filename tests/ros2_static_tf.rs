mod support;

use serial_test::serial;
use std::ffi::OsStr;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

#[cfg(feature = "ros2")]
#[test]
#[serial]
fn starts_without_tf_prompt_when_ros2_static_tf_published() {
    if !support::integration_ros_enabled() {
        eprintln!("skipping: set TERMVIZ_IT_ROS=1 to enable ROS integration tests");
        return;
    }
    if !support::command_exists("ros2") {
        eprintln!("skipping: ROS 2 CLI not found in PATH (need ros2)");
        return;
    }

    let tmp = tempfile::tempdir().unwrap();
    let cfg_path = tmp.path().join("termviz.yml");
    std::fs::write(&cfg_path, support::minimal_config_yaml()).unwrap();

    let ros_domain_id = ((SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .subsec_nanos()
        % 200)
        + 1)
    .to_string();

    let _tf = match support::spawn_ros2_static_tf(&ros_domain_id, "map", "base_link", tmp.path()) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("skipping: failed to start ROS 2 static TF publisher: {e}");
            return;
        }
    };

    let env = vec![
        ("ROS_DOMAIN_ID", ros_domain_id.as_str()),
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

    let _status = run
        .wait_with_timeout(Duration::from_secs(5))
        .unwrap_or_else(|| {
            run.kill();
            panic!(
                "termviz did not exit after Ctrl+C; output: {}",
                run.output.lock().unwrap()
            );
        });
}
