use crate::config::{ListenerConfig, ListenerConfigColor};
use crate::laser;
use crate::map;
use crate::marker;

use std::sync::Arc;

pub struct Listeners {
    pub lasers: Vec<laser::LaserListener>,
    pub markers: marker::MarkersListener,
    pub maps: Vec<map::MapListener>,
}

impl Listeners {
    pub fn new(
        tf_listener: Arc<rustros_tf::TfListener>,
        static_frame: String,
        laser_topics: Vec<ListenerConfigColor>,
        marker_topics: Vec<ListenerConfig>,
        marker_array_topics: Vec<ListenerConfig>,
        map_topics: Vec<ListenerConfigColor>,
    ) -> Listeners {
        let mut lasers: Vec<laser::LaserListener> = Vec::new();
        for laser_config in laser_topics {
            lasers.push(laser::LaserListener::new(
                laser_config,
                tf_listener.clone(),
                static_frame.clone(),
            ));
        }

        let mut markers = marker::MarkersListener::new(tf_listener.clone(), static_frame.clone());
        for marker_config in marker_topics {
            markers.add_marker_listener(&marker_config);
        }

        for m_config in marker_array_topics {
            markers.add_marker_array_listener(&m_config);
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
            lasers,
            markers,
            maps,
        }
    }
}
