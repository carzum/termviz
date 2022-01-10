use confy;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;

#[derive(Debug, Serialize, Deserialize)]
pub struct Color {
    pub r: u8,
    pub b: u8,
    pub g: u8,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct ListenerConfig {
    pub topic: String,
    pub color: Color,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct TeleopConfig {
    pub key_mapping: HashMap<String, (String, i8)>,
    pub increment: f64,
    pub cmd_vel_topic: String,
}

impl Default for TeleopConfig {
    fn default() -> TeleopConfig {
        let default_mapping = HashMap::from([
            ("w".to_string(), ("x".to_string(), 1)),
            ("s".to_string(), ("x".to_string(), -1)),
            ("q".to_string(), ("y".to_string(), 1)),
            ("e".to_string(), ("y".to_string(), -1)),
            ("a".to_string(), ("theta".to_string(), 1)),
            ("d".to_string(), ("theta".to_string(), -1)),
        ]);
        TeleopConfig {
            key_mapping: default_mapping,
            increment: 0.1,
            cmd_vel_topic: "cmd_vel".to_string(),
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct RosoutConfig {
    pub buffer_size: usize,
    pub min_loglevel: i8,
    pub screen_percentage: u16,
    pub enabled_by_default: bool,
}

impl Default for RosoutConfig {
    fn default() -> RosoutConfig {
        RosoutConfig {
            buffer_size: 150,
            min_loglevel: 2, // INFO, see rosmsg show rosgraph_msgs/Log
            screen_percentage: 25,
            enabled_by_default: true,
        }
    }
}

#[derive(Debug, Serialize, Deserialize)]
pub struct TermvizConfig {
    pub fixed_frame: String,
    pub robot_frame: String,
    pub map_topics: Vec<ListenerConfig>,
    pub laser_topics: Vec<ListenerConfig>,
    pub marker_array_topics: Vec<ListenerConfig>,
    pub target_framerate: i64,
    pub axis_length: f64,
    pub visible_area: Vec<f64>, //Borders of map from center in Meter
    pub zoom_factor: f64,
    pub teleop_key_mapping: TeleopConfig,
    pub rosout_config: RosoutConfig,
}

impl Default for TermvizConfig {
    fn default() -> Self {
        let conf = TermvizConfig {
            fixed_frame: "map".to_string(),
            robot_frame: "base_link".to_string(),
            map_topics: vec![ListenerConfig {
                topic: "map".to_string(),
                color: Color {
                    r: 255,
                    b: 255,
                    g: 255,
                },
            }],
            laser_topics: vec![ListenerConfig {
                topic: "scan".to_string(),
                color: Color { r: 200, b: 0, g: 0 },
            }],
            marker_array_topics: vec![ListenerConfig {
                topic: "marker_array".to_string(),
                color: Color {
                    r: 20,
                    b: 20,
                    g: 200,
                },
            }],
            target_framerate: 30,
            axis_length: 0.5,
            visible_area: vec![-5., 5., -5., 5.],
            zoom_factor: 0.1,
            teleop_key_mapping: TeleopConfig::default(),
            rosout_config: RosoutConfig::default(),
        };
        let res = confy::store("termviz", "termviz", &conf);
        match res {
            Ok(v) => println!("Stored default config: {:?}", v),
            Err(e) => println!("Error storing default config: {:?}", e),
        }
        conf
    }
}

pub fn get_config() -> Result<TermvizConfig, confy::ConfyError> {
    let cfg: TermvizConfig = confy::load("termviz", "termviz")?;
    println!("{:#?}", cfg);
    Ok(cfg)
}
