use crate::config::MapListenerConfig;
use crate::ros;
use crate::ros::tf::TfClient;
use crate::ros::types;
use crate::transformation;
use std::sync::{Arc, RwLock};

use nalgebra::geometry::Point3;

pub struct MapListener {
    pub config: MapListenerConfig,
    pub points: Arc<RwLock<Vec<(f64, f64)>>>,
    _tf: Arc<dyn TfClient>,
    _static_frame: String,
    _subscriber: ros::SubscriptionHandle,
}

impl MapListener {
    pub fn new(
        config: MapListenerConfig,
        tf: Arc<dyn TfClient>,
        static_frame: String,
    ) -> MapListener {
        let occ_points = Arc::new(RwLock::new(Vec::<(f64, f64)>::new()));
        let cb_occ_points = occ_points.clone();
        let str_ = static_frame.clone();
        let local_tf = tf.clone();
        let threshold = config.threshold.clone();
        let _map_sub = ros::subscribe_occupancy_grid(&config.topic, 1, move |map: types::OccupancyGrid| {
                let mut points: Vec<(f64, f64)> = Vec::new();
                let res = local_tf.lookup_transform(&str_, &map.header.frame_id, map.header.stamp);
                match &res {
                    Ok(res) => res,
                    Err(_e) => return,
                };

                let isometry = transformation::ros_pose_to_isometry(&map.info.origin);

                for (i, pt) in map.data.iter().enumerate() {
                    let line = i / map.info.width as usize;
                    let column = i - line * map.info.width as usize;
                    if pt >= &threshold {
                        let trans_point = isometry.transform_point(&Point3::new(
                            (column as f64) * map.info.resolution as f64,
                            line as f64 * map.info.resolution as f64,
                            0.,
                        ));
                        let global_point = transformation::transform_relative_pt(
                            &res.as_ref().unwrap().transform,
                            (trans_point[0], trans_point[1]),
                        );
                        points.push(global_point);
                    }
                }
                let mut cb_occ_points = cb_occ_points.write().unwrap();
                *cb_occ_points = points;
            })
            .unwrap();

        MapListener {
            config,
            points: occ_points,
            _tf: tf,
            _static_frame: static_frame.to_string(),
            _subscriber: _map_sub,
        }
    }
}
