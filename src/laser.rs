use crate::config::ListenerConfig;
use crate::transformation;
use std::sync::{Arc, Mutex, RwLock};


use rosrust;
use rustros_tf;

pub struct LaserListener {
    pub config: ListenerConfig,
    pub points: Arc<RwLock<Vec<(f64, f64)>>>,
    _tf_listener: Arc<Mutex<rustros_tf::TfListener>>,
    _static_frame: String,
    _subscriber: rosrust::Subscriber,
}

impl LaserListener {
    pub fn new(config: ListenerConfig,
           tf_listener: Arc<Mutex<rustros_tf::TfListener>>,
           static_frame: String,
           ) -> LaserListener
    {

        let scan_points = Arc::new(RwLock::new(Vec::<(f64, f64)>::new()));
        let cb_scan_points = scan_points.clone();
        let laser_listener = tf_listener.clone();
        let str_ = static_frame.clone();
        let laser_sub = rosrust::subscribe(
            &config.topic, 2, move |scan: rosrust_msg::sensor_msgs::LaserScan| {
            let mut points: Vec<(f64, f64)> = Vec::new();
            let laser_listener = laser_listener.lock().unwrap();
            let res = &laser_listener.lookup_transform(
                &str_, &scan.header.frame_id, scan.header.stamp);
            match res {
                Ok(res) => res,
                Err(_e) => return,
            };
            let tf = &res.as_ref().unwrap().transform;
            for (i, range) in scan.ranges.iter().enumerate() {
                let angle = scan.angle_min + i as f32 * scan.angle_increment;
                let pt = transformation::transform_relative_point(
                    tf, (*range as f64 * angle.cos() as f64,
                    *range as f64 * angle.sin() as f64));
                if range > &scan.range_min {
                   points.push(pt);
                }
            };
            let mut cb_scan_points = cb_scan_points.write().unwrap();
            *cb_scan_points = points;
            }).unwrap();

        LaserListener{
            config,
            points: scan_points,
            _tf_listener: tf_listener,
            _static_frame: static_frame.to_string(),
            _subscriber: laser_sub,
        }
    }
}
