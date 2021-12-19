use crate::config::ListenerConfig;
use crate::transformation;
use std::sync::{Arc, Mutex, RwLock};


use nalgebra::geometry::{Quaternion, UnitQuaternion, Point3, Isometry3, Translation3};

use rosrust;
use rustros_tf;

pub struct MapListener {
    pub config: ListenerConfig,
    pub points: Arc<RwLock<Vec<(f64, f64)>>>,
    _tf_listener: Arc<Mutex<rustros_tf::TfListener>>,
    _static_frame: String,
    _subscriber: rosrust::Subscriber,
}

impl MapListener {
    pub fn new(config: ListenerConfig,
           tf_listener: Arc<Mutex<rustros_tf::TfListener>>,
           static_frame: String,
           ) -> MapListener
    {

        let occ_points = Arc::new(RwLock::new(Vec::<(f64, f64)>::new()));
        let cb_occ_points = occ_points.clone();
        let map_listener = tf_listener.clone();
        let str_ = static_frame.clone();
        let map_sub = rosrust::subscribe(
            &config.topic, 1, move | map: rosrust_msg::nav_msgs::OccupancyGrid| {
            let mut points: Vec<(f64, f64)> = Vec::new();
            let map_listener = map_listener.lock().unwrap();
            let res = &map_listener.lookup_transform(
                &str_, &map.header.frame_id, map.header.stamp);
            match res {
                Ok(res) => res,
                Err(_e) => return,
            };
            let tf = &res.as_ref().unwrap().transform;

            let tra = Translation3::new(map.info.origin.position.x,
                                        map.info.origin.position.y,
                                        map.info.origin.position.z);
            let rot = UnitQuaternion::new_normalize(Quaternion::new(
                    map.info.origin.orientation.w,
                    map.info.origin.orientation.x,
                    map.info.origin.orientation.y,
                    map.info.origin.orientation.z,
                    ));
            let isometry = Isometry3::from_parts(tra, rot);

            for (i, pt) in map.data.iter().enumerate() {
                let line = i / map.info.width as usize;
                let column = i - line * map.info.width as usize;
                if pt != &0 {
                    let trans_point = isometry.transform_point(&Point3::new(
                            (column as f64) * map.info.resolution as f64,
                            line as f64 * map.info.resolution as f64,
                            0.));
                    let global_point = transformation::transform_relative_point(
                        tf, (trans_point[0],trans_point[1]));
                    if pt > &0 {
                        points.push(global_point);
                    }
                }
            };
            let mut cb_occ_points = cb_occ_points.write().unwrap();
            *cb_occ_points = points;
            }).unwrap();

        MapListener{
            config,
            points: occ_points,
            _tf_listener: tf_listener,
            _static_frame: static_frame.to_string(),
            _subscriber: map_sub,
        }
    }
}
