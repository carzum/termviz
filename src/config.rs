use crate::app_modes::input;
use confy;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;

fn default_int() -> i64 {
    0
}

fn default_map_threshold() -> i8 {
    1
}

fn color_white() -> Color {
    Color {
        r: 255,
        g: 255,
        b: 255,
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct Color {
    pub r: u8,
    pub b: u8,
    pub g: u8,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ListenerConfig {
    pub topic: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ImageListenerConfig {
    pub topic: String,
    #[serde(default = "default_int")]
    pub rotation: i64,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ListenerConfigColor {
    pub topic: String,
    pub color: Color,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct MapListenerConfig {
    pub topic: String,
    #[serde(default = "color_white")]
    pub color: Color,
    #[serde(default = "default_map_threshold")]
    pub threshold: i8,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct TeleopConfig {
    pub default_increment: f64,
    pub increment_step: f64,
    pub cmd_vel_topic: String,
}

impl Default for TeleopConfig {
    fn default() -> TeleopConfig {
        TeleopConfig {
            default_increment: 0.1,
            increment_step: 0.1,
            cmd_vel_topic: "cmd_vel".to_string(),
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct TermvizConfig {
    pub fixed_frame: String,
    pub robot_frame: String,
    pub map_topics: Vec<MapListenerConfig>,
    pub laser_topics: Vec<ListenerConfigColor>,
    pub marker_topics: Vec<ListenerConfig>,
    pub image_topics: Vec<ImageListenerConfig>,
    pub marker_array_topics: Vec<ListenerConfig>,
    pub send_pose_topic: String,
    pub target_framerate: i64,
    pub axis_length: f64,
    pub visible_area: Vec<f64>, //Borders of map from center in Meter
    pub zoom_factor: f64,
    pub key_mapping: HashMap<String, String>,
    pub teleop: TeleopConfig,
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
            send_pose_topic: "initialpose".to_string(),
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
                (input::SHOW_HELP.to_string(), "h".to_string()),
                (input::MODE_2.to_string(), "t".to_string()),
                (input::MODE_3.to_string(), "i".to_string()),
            ]),
            teleop: TeleopConfig::default(),
        }
    }
}

pub fn get_config() -> Result<TermvizConfig, confy::ConfyError> {
    let mut cfg = TermvizConfig::default();
    let user_path = confy::get_configuration_file_path("termviz", "termviz")?;
    if Path::new(&user_path).exists() {
        // use user config if exists
        cfg = confy::load("termviz", "termviz")?;
    } else {
        let sys_path = "/etc/termviz/termviz.yml";
        if Path::new(sys_path).exists() {
            // fallback to system config
            cfg = confy::load_path(sys_path)?;
        } else {
            let res = confy::store("termviz", "termviz", &cfg);
            match res {
                Ok(v) => println!("Stored default config: {:?}", v),
                Err(e) => println!("Error storing default config: {:?}", e),
            }
        }
    };
    Ok(cfg)
}
