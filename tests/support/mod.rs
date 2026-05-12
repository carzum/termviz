use portable_pty::{native_pty_system, CommandBuilder, PtySize};
use std::io::{Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::path::Path;
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use std::{ffi::OsStr, io};

pub fn termviz_bin() -> PathBuf {
    std::env::var_os("CARGO_BIN_EXE_termviz")
        .map(PathBuf::from)
        .or_else(|| {
            std::env::current_exe()
                .ok()
                .and_then(|path| path.parent().map(Path::to_path_buf))
                .and_then(|path| path.parent().map(Path::to_path_buf))
                .map(|path| path.join("termviz"))
                .filter(|path| path.is_file())
        })
        .expect("termviz binary not found; run tests via `cargo test` so Cargo builds the binary")
}

pub fn minimal_config_yaml() -> &'static str {
    // Minimal config so `confy::load_path` succeeds.
    // Keep topic lists mostly empty to avoid requiring message types.
    // Intentionally omits some `teleop` fields to exercise defaults.
    r#"
fixed_frame: map
robot_frame: base_link

map_topics:
  - topic: map
    color:
      r: 255
      g: 255
      b: 255
    threshold: 1
laser_topics: []
marker_topics: []
image_topics: []
marker_array_topics: []
path_topics: []
pointcloud2_topics: []
polygon_stamped_topics: []
pose_array_topics: []
pose_stamped_topics: []
send_pose_topics:
  - topic: initialpose
    msg_type: PoseWithCovarianceStamped

target_framerate: 10
axis_length: 0.5
visible_area: [ -5.0, 5.0, -5.0, 5.0 ]
zoom_factor: 0.1

key_mapping:
    {}

teleop:
  default_increment: 0.1
  increment_step: 0.1
  cmd_vel_topic: cmd_vel
  mode: Safe
  max_vel: 0.2
"#
}

pub fn integration_ros_enabled() -> bool {
    matches!(std::env::var("TERMVIZ_IT_ROS").as_deref(), Ok("1"))
}

pub fn command_exists(cmd: &str) -> bool {
    Command::new(cmd)
        .arg("--help")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .is_ok()
}

#[allow(dead_code)]
pub fn pick_unused_port() -> u16 {
    std::net::TcpListener::bind("127.0.0.1:0")
        .expect("bind ephemeral port")
        .local_addr()
        .expect("local addr")
        .port()
}

fn wait_for_tcp<A: ToSocketAddrs + Copy>(addr: A, timeout: Duration) -> bool {
    let start = Instant::now();
    while start.elapsed() < timeout {
        if TcpStream::connect(addr).is_ok() {
            return true;
        }
        thread::sleep(Duration::from_millis(50));
    }
    false
}

pub struct ProcessGuard {
    child: Child,
}

impl ProcessGuard {
    pub fn spawn(mut cmd: Command) -> io::Result<Self> {
        let child = cmd.spawn()?;
        Ok(Self { child })
    }
}

impl Drop for ProcessGuard {
    fn drop(&mut self) {
        let _ = self.child.kill();
        let _ = self.child.wait();
    }
}

#[allow(dead_code)]
pub fn spawn_roscore(port: u16, log_dir: &Path) -> io::Result<ProcessGuard> {
    let mut cmd = Command::new("roscore");
    cmd.arg("-p")
        .arg(port.to_string())
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .env("ROS_LOG_DIR", log_dir);

    let guard = ProcessGuard::spawn(cmd)?;
    if !wait_for_tcp(("127.0.0.1", port), Duration::from_secs(5)) {
        return Err(io::Error::new(
            io::ErrorKind::TimedOut,
            "roscore did not start listening",
        ));
    }
    Ok(guard)
}

#[allow(dead_code)]
pub fn spawn_static_tf(
    port: u16,
    ros_master_uri: &str,
    frame: &str,
    child_frame: &str,
    log_dir: &Path,
) -> io::Result<ProcessGuard> {
    let mut cmd = Command::new("rosrun");
    cmd.arg("tf2_ros")
        .arg("static_transform_publisher")
        // x y z yaw pitch roll
        .arg("0")
        .arg("0")
        .arg("0")
        .arg("0")
        .arg("0")
        .arg("0")
        .arg(frame)
        .arg(child_frame)
        .arg("100")
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .env("ROS_MASTER_URI", ros_master_uri)
        .env("ROS_IP", "127.0.0.1")
        .env("ROS_LOG_DIR", log_dir);

    let guard = ProcessGuard::spawn(cmd)?;
    // Give the node a moment to register and start publishing
    thread::sleep(Duration::from_millis(250));
    // Ensure master is still up
    if !wait_for_tcp(("127.0.0.1", port), Duration::from_secs(2)) {
        return Err(io::Error::new(
            io::ErrorKind::TimedOut,
            "ROS master not reachable after starting static TF",
        ));
    }
    Ok(guard)
}

#[allow(dead_code)]
pub fn spawn_ros2_static_tf(
    ros_domain_id: &str,
    frame: &str,
    child_frame: &str,
    log_dir: &Path,
) -> io::Result<ProcessGuard> {
    let mut cmd = Command::new("ros2");
    cmd.arg("run")
        .arg("tf2_ros")
        .arg("static_transform_publisher")
        .arg("--x")
        .arg("0")
        .arg("--y")
        .arg("0")
        .arg("--z")
        .arg("0")
        .arg("--yaw")
        .arg("0")
        .arg("--pitch")
        .arg("0")
        .arg("--roll")
        .arg("0")
        .arg("--frame-id")
        .arg(frame)
        .arg("--child-frame-id")
        .arg(child_frame)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .env("ROS_DOMAIN_ID", ros_domain_id)
        .env("ROS_LOG_DIR", log_dir);

    let guard = ProcessGuard::spawn(cmd)?;
    thread::sleep(Duration::from_millis(500));
    Ok(guard)
}

pub struct PtyRun {
    pub child: Box<dyn portable_pty::Child + Send>,
    pub output: Arc<Mutex<String>>,
    pub writer: Box<dyn Write + Send>,
}

impl PtyRun {
    pub fn wait_for_output_contains(&self, needle: &str, timeout: Duration) -> bool {
        let start = Instant::now();
        while start.elapsed() < timeout {
            if self.output.lock().unwrap().contains(needle) {
                return true;
            }
            thread::sleep(Duration::from_millis(25));
        }
        false
    }

    pub fn send(&mut self, s: &str) {
        let _ = self.writer.write_all(s.as_bytes());
        let _ = self.writer.flush();
    }

    pub fn send_ctrl_c(&mut self) {
        self.send("\u{3}");
    }

    pub fn wait_with_timeout(&mut self, timeout: Duration) -> Option<portable_pty::ExitStatus> {
        let start = Instant::now();
        while start.elapsed() < timeout {
            match self.child.try_wait() {
                Ok(Some(status)) => return Some(status),
                Ok(None) => thread::sleep(Duration::from_millis(25)),
                Err(_) => return None,
            }
        }
        None
    }

    pub fn kill(&mut self) {
        let _ = self.child.kill();
    }
}

pub fn spawn_termviz_pty<I, S>(args: I, env: &[(&str, &str)]) -> std::io::Result<PtyRun>
where
    I: IntoIterator<Item = S>,
    S: AsRef<OsStr>,
{
    let pty = native_pty_system();
    let pair = pty
        .openpty(PtySize {
        rows: 24,
        cols: 80,
        pixel_width: 0,
        pixel_height: 0,
        })
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?;

    let mut cmd = CommandBuilder::new(termviz_bin());
    for a in args {
        cmd.arg(a.as_ref());
    }
    for (k, v) in env {
        cmd.env(k, v);
    }

    let child = pair
        .slave
        .spawn_command(cmd)
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?;

    let output = Arc::new(Mutex::new(String::new()));
    let output_clone = Arc::clone(&output);

    let mut reader = pair
        .master
        .try_clone_reader()
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?;
    thread::spawn(move || {
        let mut buf = [0u8; 8192];
        loop {
            match reader.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => {
                    let chunk = String::from_utf8_lossy(&buf[..n]);
                    output_clone.lock().unwrap().push_str(&chunk);
                }
                Err(_) => break,
            }
        }
    });

    let writer = pair
        .master
        .take_writer()
        .map_err(|e| io::Error::new(io::ErrorKind::Other, e.to_string()))?;

    Ok(PtyRun {
        child,
        output,
        writer,
    })
}
