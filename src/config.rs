use crate::app_modes::input;
use confy;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::PathBuf;
use strum_macros::Display;
use tui::style::Color as TuiColor;

const DEFAULT_PATH: &str = "/etc/termviz/termviz.yml";

const fn default_int() -> i64 {
    0
}

const fn default_map_threshold() -> i8 {
    1
}

const fn default_pose_length() -> f64 {
    0.2
}

const fn default_teleop_max_vel() -> f64 {
    0.2
}

const fn default_teleop_increment() -> f64 {
    0.1
}

const fn default_teleop_increment_step() -> f64 {
    0.1
}

fn default_teleop_cmd_vel_topic() -> String {
    "cmd_vel".to_string()
}

const fn default_publish_cmd_vel_when_idle() -> bool {
    true
}

fn default_fixed_frame() -> String {
    "map".to_string()
}

fn default_robot_frame() -> String {
    "base_link".to_string()
}

fn default_map_topics() -> Vec<MapListenerConfig> {
    vec![MapListenerConfig {
        topic: "map".to_string(),
        color: color_white(),
        threshold: default_map_threshold(),
    }]
}

fn default_laser_topics() -> Vec<ListenerConfigColor> {
    vec![ListenerConfigColor {
        topic: "scan".to_string(),
        color: Color { r: 200, g: 0, b: 0 },
    }]
}

fn default_marker_topics() -> Vec<ListenerConfig> {
    vec![ListenerConfig {
        topic: "marker".to_string(),
    }]
}

fn default_image_topics() -> Vec<ImageListenerConfig> {
    vec![ImageListenerConfig {
        topic: "image_rect".to_string(),
        rotation: default_int(),
    }]
}

fn default_marker_array_topics() -> Vec<ListenerConfig> {
    vec![ListenerConfig {
        topic: "marker_array".to_string(),
    }]
}

fn default_path_topics() -> Vec<PoseListenerConfig> {
    vec![PoseListenerConfig {
        topic: "path".to_string(),
        style: "line".to_string(),
        color: Color { r: 0, g: 255, b: 0 },
        length: default_pose_length(),
    }]
}

fn default_pointcloud2_topics() -> Vec<PointCloud2ListenerConfig> {
    vec![PointCloud2ListenerConfig {
        topic: "pointcloud2".to_string(),
        use_rgb: false,
    }]
}

fn default_polygon_stamped_topics() -> Vec<ListenerConfigColor> {
    vec![ListenerConfigColor {
        topic: "footprint".to_string(),
        color: Color { r: 200, g: 0, b: 0 },
    }]
}

fn default_pose_array_topics() -> Vec<PoseListenerConfig> {
    vec![PoseListenerConfig {
        topic: "pose_array".to_string(),
        style: "arrow".to_string(),
        color: color_red(),
        length: default_pose_length(),
    }]
}

fn default_pose_stamped_topics() -> Vec<PoseListenerConfig> {
    vec![PoseListenerConfig {
        topic: "pose_stamped".to_string(),
        style: "axis".to_string(),
        color: color_red(),
        length: default_pose_length(),
    }]
}

fn default_send_pose_topics() -> Vec<SendPoseConfig> {
    vec![SendPoseConfig {
        topic: "initialpose".to_string(),
        msg_type: "PoseWithCovarianceStamped".to_string(),
    }]
}

const fn default_target_framerate() -> i64 {
    30
}

const fn default_axis_length() -> f64 {
    0.5
}

fn default_visible_area() -> Vec<f64> {
    vec![-5.0, 5.0, -5.0, 5.0]
}

const fn default_zoom_factor() -> f64 {
    0.1
}

fn default_key_mapping() -> HashMap<String, String> {
    HashMap::from([
        (input::UP.to_string(), "w".to_string()),
        (input::DOWN.to_string(), "s".to_string()),
        (input::LEFT.to_string(), "a".to_string()),
        (input::RIGHT.to_string(), "d".to_string()),
        (input::ROTATE_LEFT.to_string(), "q".to_string()),
        (input::ROTATE_RIGHT.to_string(), "e".to_string()),
        (input::CANCEL.to_string(), "Esc".to_string()),
        (input::CONFIRM.to_string(), "Enter".to_string()),
        (input::ZOOM_IN.to_string(), "=".to_string()),
        (input::ZOOM_OUT.to_string(), "-".to_string()),
        (input::INCREMENT_STEP.to_string(), "k".to_string()),
        (input::DECREMENT_STEP.to_string(), "j".to_string()),
        (input::NEXT.to_string(), "n".to_string()),
        (input::PREVIOUS.to_string(), "b".to_string()),
        (input::SHOW_HELP.to_string(), "h".to_string()),
        (input::MODE_2.to_string(), "t".to_string()),
        (input::MODE_3.to_string(), "i".to_string()),
    ])
}

const fn color_white() -> Color {
    Color {
        r: 255,
        g: 255,
        b: 255,
    }
}

const fn color_red() -> Color {
    Color { r: 255, g: 0, b: 0 }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Color {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Color {
    pub fn to_tui(&self) -> TuiColor {
        return TuiColor::Rgb(self.r, self.g, self.b);
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ListenerConfig {
    pub topic: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PointCloud2ListenerConfig {
    pub topic: String,
    #[serde(default = "bool::default")]
    pub use_rgb: bool,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct PoseListenerConfig {
    pub topic: String,
    pub style: String,
    #[serde(default = "color_red")]
    pub color: Color,
    #[serde(default = "default_pose_length")]
    pub length: f64,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ImageListenerConfig {
    pub topic: String,
    #[serde(default = "default_int")]
    pub rotation: i64,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct SendPoseConfig {
    pub topic: String,
    pub msg_type: String,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct ListenerConfigColor {
    pub topic: String,
    pub color: Color,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct MapListenerConfig {
    pub topic: String,
    #[serde(default = "color_white")]
    pub color: Color,
    #[serde(default = "default_map_threshold")]
    pub threshold: i8,
}

#[derive(Debug, Default, Serialize, Deserialize, Clone, PartialEq, Eq, Display)]
pub enum TeleopMode {
    // Node will keep speed button press only increment decrement
    Classic,
    // Speed will be reduced every iteration by one increment
    // To keep driving a button needs to be pressed every increment
    #[default]
    Safe,
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(default)]
pub struct TeleopConfig {
    #[serde(default = "default_teleop_increment")]
    pub default_increment: f64,
    #[serde(default = "default_teleop_increment_step")]
    pub increment_step: f64,
    #[serde(default = "default_teleop_cmd_vel_topic")]
    pub cmd_vel_topic: String,
    #[serde(default = "default_publish_cmd_vel_when_idle")]
    pub publish_cmd_vel_when_idle: bool,
    #[serde(default)]
    pub mode: TeleopMode,
    #[serde(default = "default_teleop_max_vel")]
    pub max_vel: f64,
}

impl Default for TeleopConfig {
    fn default() -> TeleopConfig {
        TeleopConfig {
            default_increment: default_teleop_increment(),
            increment_step: default_teleop_increment_step(),
            cmd_vel_topic: default_teleop_cmd_vel_topic(),
            publish_cmd_vel_when_idle: default_publish_cmd_vel_when_idle(),
            mode: TeleopMode::default(),
            max_vel: default_teleop_max_vel(),
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
#[serde(default)]
pub struct TermvizConfig {
    #[serde(default = "default_fixed_frame")]
    pub fixed_frame: String,
    #[serde(default = "default_robot_frame")]
    pub robot_frame: String,
    #[serde(default = "default_map_topics")]
    pub map_topics: Vec<MapListenerConfig>,
    #[serde(default = "default_laser_topics")]
    pub laser_topics: Vec<ListenerConfigColor>,
    #[serde(default = "default_marker_topics")]
    pub marker_topics: Vec<ListenerConfig>,
    #[serde(default = "default_image_topics")]
    pub image_topics: Vec<ImageListenerConfig>,
    #[serde(default = "default_marker_array_topics")]
    pub marker_array_topics: Vec<ListenerConfig>,
    #[serde(default = "default_path_topics")]
    pub path_topics: Vec<PoseListenerConfig>,
    #[serde(default = "default_pointcloud2_topics")]
    pub pointcloud2_topics: Vec<PointCloud2ListenerConfig>,
    #[serde(default = "default_polygon_stamped_topics")]
    pub polygon_stamped_topics: Vec<ListenerConfigColor>,
    #[serde(default = "default_pose_array_topics")]
    pub pose_array_topics: Vec<PoseListenerConfig>,
    #[serde(default = "default_pose_stamped_topics")]
    pub pose_stamped_topics: Vec<PoseListenerConfig>,
    #[serde(default = "default_send_pose_topics")]
    pub send_pose_topics: Vec<SendPoseConfig>,
    #[serde(default = "default_target_framerate")]
    pub target_framerate: i64,
    #[serde(default = "default_axis_length")]
    pub axis_length: f64,
    #[serde(default = "default_visible_area")]
    pub visible_area: Vec<f64>, //Borders of map from center in Meter
    #[serde(default = "default_zoom_factor")]
    pub zoom_factor: f64,
    #[serde(default = "default_key_mapping")]
    pub key_mapping: HashMap<String, String>,
    #[serde(default)]
    pub teleop: TeleopConfig,
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;
    use tempfile::NamedTempFile;

    fn write_config(contents: &str) -> NamedTempFile {
        let mut file = NamedTempFile::new().expect("temp file");
        file.write_all(contents.as_bytes()).expect("write yaml");
        file
    }

    #[test]
    fn loads_old_yaml_missing_new_teleop_field() {
        // Intentionally omit `teleop.publish_cmd_vel_when_idle` to verify backward compat.
        // Also omit `pointcloud2_topics` to verify newly-added top-level fields don't break older configs.
        let yaml = r#"
fixed_frame: map
robot_frame: base_link

map_topics:
-
    topic: map
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
polygon_stamped_topics: []
pose_array_topics: []
pose_stamped_topics: []
send_pose_topics:
-
    topic: initialpose
    msg_type: PoseWithCovarianceStamped

target_framerate: 10
axis_length: 0.5
visible_area: [ -5.0, 5.0, -5.0, 5.0 ]
zoom_factor: 0.1

key_mapping: {}

teleop:
    default_increment: 0.1
    increment_step: 0.1
    cmd_vel_topic: cmd_vel
    mode: Safe
    max_vel: 0.2
"#;

        let file = write_config(yaml);
        let cfg: TermvizConfig = confy::load_path(file.path()).expect("load config");
        assert!(cfg.teleop.publish_cmd_vel_when_idle);
        assert!(!cfg.pointcloud2_topics.is_empty());
        assert_eq!(cfg.pointcloud2_topics[0].topic, "pointcloud2");
    }

    #[test]
    fn loads_yaml_without_teleop_block() {
        let yaml = r#"
fixed_frame: map
robot_frame: base_link

map_topics:
-
    topic: map
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
-
    topic: initialpose
    msg_type: PoseWithCovarianceStamped

target_framerate: 10
axis_length: 0.5
visible_area: [ -5.0, 5.0, -5.0, 5.0 ]
zoom_factor: 0.1

key_mapping: {}
"#;

        let file = write_config(yaml);
        let cfg: TermvizConfig = confy::load_path(file.path()).expect("load config");

        let default = TeleopConfig::default();
        assert_eq!(cfg.teleop.default_increment, default.default_increment);
        assert_eq!(cfg.teleop.increment_step, default.increment_step);
        assert_eq!(cfg.teleop.cmd_vel_topic, default.cmd_vel_topic);
        assert_eq!(
            cfg.teleop.publish_cmd_vel_when_idle,
            default.publish_cmd_vel_when_idle
        );
        assert_eq!(cfg.teleop.mode, default.mode);
        assert_eq!(cfg.teleop.max_vel, default.max_vel);
    }
}

impl Default for TermvizConfig {
    fn default() -> Self {
        TermvizConfig {
            fixed_frame: default_fixed_frame(),
            robot_frame: default_robot_frame(),
            map_topics: default_map_topics(),
            laser_topics: default_laser_topics(),
            marker_array_topics: default_marker_array_topics(),
            marker_topics: default_marker_topics(),
            image_topics: default_image_topics(),
            pose_stamped_topics: default_pose_stamped_topics(),
            pose_array_topics: default_pose_array_topics(),
            path_topics: default_path_topics(),
            pointcloud2_topics: default_pointcloud2_topics(),
            polygon_stamped_topics: default_polygon_stamped_topics(),
            send_pose_topics: default_send_pose_topics(),
            target_framerate: default_target_framerate(),
            axis_length: default_axis_length(),
            visible_area: default_visible_area(),
            zoom_factor: default_zoom_factor(),
            key_mapping: default_key_mapping(),
            teleop: TeleopConfig::default(),
        }
    }
}

/// Loads the config with the following priority:
/// 1. from a custom path, if provided;
/// 2. from the default user config path, if it exists;
/// 3. from the default global config path, if it exists;
/// 4. or with default values.
pub fn get_config(custom_path: Option<&PathBuf>) -> Result<TermvizConfig, confy::ConfyError> {
    let default_path = PathBuf::from(DEFAULT_PATH);
    let user_path = confy::get_configuration_file_path("termviz", "termviz")?;

    let load_config_path = custom_path.unwrap_or_else(|| {
        if user_path.exists() {
            &user_path
        } else {
            &default_path
        }
    });

    let cfg = if load_config_path.exists() {
        println!("Loading config from: {:?}", load_config_path);
        confy::load_path(&load_config_path)?
    } else {
        println!("No config found at {:?}, using defaults.", load_config_path);
        TermvizConfig::default()
    };

    // Always update user config, unless a custom path was used.
    if custom_path.is_none() {
        match confy::store_path(&user_path.as_path(), &cfg) {
            Ok(_) => println!("Updated {:?}", user_path),
            Err(e) => println!("Error updating config at user path: {:?}", e),
        }
    }
    Ok(cfg)
}
