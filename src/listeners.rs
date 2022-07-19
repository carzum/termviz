use crate::config::{ListenerConfig, ListenerConfigColor, MapListenerConfig, PoseListenerConfig, PointCloud2ListenerConfig};
use crate::laser;
use crate::map;
use crate::marker;
use crate::pose;
use crate::pointcloud;

use std::sync::Arc;

pub struct Listeners {
    pub lasers: Vec<laser::LaserListener>,
    pub markers: marker::MarkersListener,
    pub maps: Vec<map::MapListener>,
    pub pose_stamped: Vec<pose::PoseStampedListener>,
    pub pose_array: Vec<pose::PoseArrayListener>,
    pub pointclouds: Vec<pointcloud::PointCloud2Listener>,
    pub paths: Vec<pose::PathListener>,
}

impl Listeners {
    pub fn new(
        tf_listener: Arc<rustros_tf::TfListener>,
        static_frame: String,
        laser_topics: Vec<ListenerConfigColor>,
        marker_topics: Vec<ListenerConfig>,
        marker_array_topics: Vec<ListenerConfig>,
        map_topics: Vec<MapListenerConfig>,
        pose_stamped_topics: Vec<PoseListenerConfig>,
        pose_array_topics: Vec<PoseListenerConfig>,
        pointcloud2_topics: Vec<PointCloud2ListenerConfig>,
        path_topics: Vec<PoseListenerConfig>,
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

        let mut pointclouds: Vec<pointcloud::PointCloud2Listener> = Vec::new();
        for pc_config in pointcloud2_topics {
            pointclouds.push(pointcloud::PointCloud2Listener::new(
                pc_config,
                tf_listener.clone(),
                static_frame.clone(),
            ));
        }

        let pose_stamped = pose_stamped_topics
            .into_iter()
            .map(|topic| pose::PoseStampedListener::new(topic))
            .collect();
        let pose_array = pose_array_topics
            .into_iter()
            .map(|topic| pose::PoseArrayListener::new(topic))
            .collect();
        let paths = path_topics
            .into_iter()
            .map(|topic| pose::PathListener::new(topic))
            .collect();
        Listeners {
            lasers,
            markers,
            maps,
            pose_stamped,
            pose_array,
            pointclouds,
            paths,
        }
    }
}
