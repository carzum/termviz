use confy;
use serde::{Serialize, Deserialize};

#[derive(Debug, Serialize, Deserialize)]
pub struct TermvizConfig {
    pub fixed_frame: String,
    pub camera_frame: String,
    pub map_topics: Vec<String>,
    pub laser_topics: Vec<String>,
    pub marker_array_topics: Vec<String>,
    pub target_framerate: i64,
}

impl Default for TermvizConfig {
    fn default() -> Self {
        let conf= TermvizConfig {
            fixed_frame: "map".to_string(),
            camera_frame: "base_link".to_string(),
            map_topics: vec!["map".to_string()],
            laser_topics: vec!["scan".to_string()],
            marker_array_topics: vec!["marker_array".to_string()],
            target_framerate: 42,
        };
        let res = confy::store("termviz", &conf);
        match res {
            Ok(v) => println!("Stored default config: {:?}", v),
            Err(e) => println!("Error storing default config: {:?}", e),
        }
        conf
    }
}

pub fn get_config() -> Result<TermvizConfig, confy::ConfyError> {
    let cfg: TermvizConfig = confy::load("termviz")?;
    println!("{:#?}", cfg);
    Ok(cfg)
}
