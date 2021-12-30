use crate::config::ListenerConfig;
use crate::laser;
use crate::map;
use crate::marker;

use std::sync::Arc;

pub struct Listeners {
    pub lasers: Vec<laser::LaserListener>,
    pub markers: Vec<marker::MarkerListener>,
    pub maps: Vec<map::MapListener>,
}

impl Listeners {
    pub fn new(
        tf_listener: Arc<rustros_tf::TfListener>,
        static_frame: String,
        laser_topics: Vec<ListenerConfig>,
        marker_topics: Vec<ListenerConfig>,
        map_topics: Vec<ListenerConfig>,
    ) -> Listeners {
        let mut lasers: Vec<laser::LaserListener> = Vec::new();
        for laser_config in laser_topics {
            lasers.push(laser::LaserListener::new(
                laser_config,
                tf_listener.clone(),
                static_frame.clone(),
            ));
        }

        let mut markers: Vec<marker::MarkerListener> = Vec::new();
        for marker_config in marker_topics {
            markers.push(marker::MarkerListener::new(
                marker_config,
                tf_listener.clone(),
                static_frame.clone(),
            ));
        }

        let mut maps: Vec<map::MapListener> = Vec::new();
        for map_config in map_topics {
            maps.push(map::MapListener::new(
                map_config,
                tf_listener.clone(),
                static_frame.clone(),
            ));
        }

        Listeners {
            lasers: lasers,
            markers: markers,
            maps: maps,
        }
    }
}
