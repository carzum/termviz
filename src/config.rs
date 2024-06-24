use crate::app_modes::input;
use confy;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::io;
use std::io::Write;
use std::path::Path;
use tui::style::Color as TuiColor;

fn default_int() -> i64 {
    0
}

fn default_map_threshold() -> i8 {
    1
}

fn default_pose_length() -> f64 {
    0.2
}

fn color_white() -> Color {
    Color {
        r: 255,
        g: 255,
        b: 255,
    }
}

fn color_red() -> Color {
    Color { r: 255, g: 0, b: 0 }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct Color {
    pub r: u8,
    pub b: u8,
    pub g: u8,
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

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct TeleopConfig {
    pub default_increment: f64,
    pub increment_step: f64,
    pub cmd_vel_topic: String,
    pub publish_cmd_vel_when_idle: bool,
}

impl Default for TeleopConfig {
    fn default() -> TeleopConfig {
        TeleopConfig {
            default_increment: 0.1,
            increment_step: 0.1,
            cmd_vel_topic: "cmd_vel".to_string(),
            publish_cmd_vel_when_idle: true,
        }
    }
}

#[derive(Debug, Serialize, Deserialize, Clone)]
pub struct TermvizConfig {
    pub fixed_frame: String,
    pub robot_frame: String,
    pub map_topics: Vec<MapListenerConfig>,
    pub laser_topics: Vec<ListenerConfigColor>,
    pub marker_topics: Vec<ListenerConfig>,
    pub image_topics: Vec<ImageListenerConfig>,
    pub marker_array_topics: Vec<ListenerConfig>,
    pub path_topics: Vec<PoseListenerConfig>,
    pub pointcloud2_topics: Vec<PointCloud2ListenerConfig>,
    pub polygon_stamped_topics: Vec<ListenerConfigColor>,
    pub pose_array_topics: Vec<PoseListenerConfig>,
    pub pose_stamped_topics: Vec<PoseListenerConfig>,
    pub send_pose_topics: Vec<SendPoseConfig>,
    pub target_framerate: i64,
    pub axis_length: f64,
    pub visible_area: Vec<f64>, //Borders of map from center in Meter
    pub zoom_factor: f64,
    pub key_mapping: HashMap<String, String>,
    pub teleop: TeleopConfig,
    pub tf_frames_service_name: String,
}

impl Default for TermvizConfig {
    fn default() -> Self {
        TermvizConfig {
            fixed_frame: "map".to_string(),
            robot_frame: "base_link".to_string(),
            map_topics: vec![MapListenerConfig {
                topic: "map".to_string(),
                color: Color {
                    r: 255,
                    b: 255,
                    g: 255,
                },
                threshold: 1,
            }],
            laser_topics: vec![ListenerConfigColor {
                topic: "scan".to_string(),
                color: Color { r: 200, b: 0, g: 0 },
            }],
            marker_array_topics: vec![ListenerConfig {
                topic: "marker_array".to_string(),
            }],
            marker_topics: vec![ListenerConfig {
                topic: "marker".to_string(),
            }],
            image_topics: vec![ImageListenerConfig {
                topic: "image_rect".to_string(),
                rotation: 0,
            }],
            pose_stamped_topics: vec![PoseListenerConfig {
                topic: "pose_stamped".to_string(),
                style: "axis".to_string(),
                color: Color { r: 255, g: 0, b: 0 },
                length: 0.2,
            }],
            pose_array_topics: vec![PoseListenerConfig {
                topic: "pose_array".to_string(),
                style: "arrow".to_string(),
                color: Color { r: 255, g: 0, b: 0 },
                length: 0.2,
            }],
            path_topics: vec![PoseListenerConfig {
                topic: "path".to_string(),
                style: "line".to_string(),
                color: Color { r: 0, g: 255, b: 0 },
                length: 0.2,
            }],
            pointcloud2_topics: vec![PointCloud2ListenerConfig {
                topic: "pointcloud2".to_string(),
                use_rgb: false,
            }],
            polygon_stamped_topics: vec![ListenerConfigColor {
                topic: "footprint".to_string(),
                color: Color { r: 200, b: 0, g: 0 },
            }],
            send_pose_topics: vec![SendPoseConfig {
                topic: "initialpose".to_string(),
                msg_type: "PoseWithCovarianceStamped".to_string(),
            }],
            target_framerate: 30,
            axis_length: 0.5,
            visible_area: vec![-5., 5., -5., 5.],
            zoom_factor: 0.1,
            key_mapping: HashMap::from([
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
                (input::UPDATE.to_string(), "u".to_string()),
                (input::SHOW_HELP.to_string(), "h".to_string()),
                (input::MODE_2.to_string(), "t".to_string()),
                (input::MODE_3.to_string(), "i".to_string()),
            ]),
            teleop: TeleopConfig::default(),
            tf_frames_service_name: "tf_service/tf2_frames".to_string(),
        }
    }
}

pub fn ask_store() -> bool {
    let mut x = String::with_capacity(5);
    print!("Store default config? (y|N): ");
    let _ = io::stdout().flush();
    io::stdin().read_line(&mut x).expect("Error reading input");
    let x = x.strip_suffix("\n").unwrap();
    match x.to_lowercase().as_str() {
        "y" | "yes" => true,
        _ => false,
    }
}

pub fn get_config(config_path: Option<&String>) -> Result<TermvizConfig, confy::ConfyError> {
    let user_path = confy::get_configuration_file_path("termviz", "termviz")?;

    // decide on which config path to load
    let load_config_path = if config_path.is_some() {
        // config path provided by command line arg
        Path::new(config_path.unwrap())
    } else if user_path.as_path().exists() {
        // use user config if it exists
        user_path.as_path()
    } else {
        // fallback to system config
        Path::new("/etc/termviz/termviz.yml")
    };

    let mut cfg = TermvizConfig::default();
    if load_config_path.exists() {
        println!("Loading config from: {:?}", load_config_path);
        cfg = confy::load_path(load_config_path)?;
    } else {
        // no config found, generate default
        println!("No config found, using default");
        let store = ask_store();
        if store {
            let res = confy::store("termviz", "termviz", &cfg);
            match res {
                Ok(_) => println!("Stored default config at {:?}", user_path),
                Err(e) => println!("Error storing default config: {:?}", e),
            }
        }
    };
    Ok(cfg)
}
