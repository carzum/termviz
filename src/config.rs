use confy;
use serde::{Serialize, Deserialize};

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
pub struct TermvizConfig {
    pub fixed_frame: String,
    pub camera_frame: String,
    pub map_topics: Vec<ListenerConfig>,
    pub laser_topics: Vec<ListenerConfig>,
    pub marker_array_topics: Vec<ListenerConfig>,
    pub target_framerate: i64,
    pub axis_length: f64,
    pub visible_area: Vec<f64>, //Borders of map from center in Meter
    pub zoom_factor: f64,
}

impl Default for TermvizConfig {
    fn default() -> Self {
        let conf= TermvizConfig {
            fixed_frame: "map".to_string(),
            camera_frame: "base_link".to_string(),
            map_topics: vec![ ListenerConfig {
                topic: "map".to_string(),
                color: Color { r: 255, b: 255, g: 255 } }],
            laser_topics: vec![ListenerConfig {
                topic: "scan".to_string(),
                color: Color { r: 200, b: 0, g: 0 } }],
            marker_array_topics: vec![ListenerConfig {
                topic: "marker_array".to_string(),
                color: Color { r: 20, b: 20, g: 200 } }],
            target_framerate: 30,
            axis_length: 0.5,
            visible_area: vec![-5., 5., -5., 5.],
            zoom_factor: 0.1,
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
