use crate::laser;
use crate::marker;
use crate::map;

use std::sync::{Arc, Mutex};


pub struct Listeners {
    pub lasers: Vec<laser::LaserListener>,
    pub markers: Vec<marker::MarkerListener>,
    pub maps: Vec<map::MapListener>,
}

impl Listeners {
    pub fn new(
        tf_listener: Arc<Mutex<rustros_tf::TfListener>>,
        static_frame: String,
        laser_topics: Vec<String>,
        marker_topics: Vec<String>,
        map_topics: Vec<String>,
        ) -> Listeners
    {
        let mut lasers: Vec<laser::LaserListener> = Vec::new();
        for laser_topic in laser_topics {
            lasers.push(
                laser::LaserListener::new(
                    &laser_topic, tf_listener.clone(),
                    static_frame.clone()));
        }

        let mut markers: Vec<marker::MarkerListener> = Vec::new();
        for marker_topic in marker_topics {
            markers.push(
                marker::MarkerListener::new(
                    &marker_topic, tf_listener.clone(),
                    static_frame.clone()));
        }

        let mut maps: Vec<map::MapListener> = Vec::new();
        for map_topic in map_topics {
            maps.push(
                map::MapListener::new(
                    &map_topic, tf_listener.clone(),
                    static_frame.clone()));
        }

        Listeners{
            lasers: lasers,
            markers: markers,
            maps: maps
        }
    }

}
