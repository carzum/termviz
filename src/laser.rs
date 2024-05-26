use crate::{config::ListenerConfigColor, transformation::ros_transform_to_isometry};
use crate::transformation;
use std::sync::{Arc, RwLock};

use nalgebra::Isometry3;
use rosrust;
use rustros_tf;
use rustros_tf::transforms::isometry_to_transform;

pub struct LaserListener {
    pub config: ListenerConfigColor,
    pub points: Arc<RwLock<Vec<(f64, f64)>>>,
    _tf_listener: Arc<rustros_tf::TfListener>,
    _static_frame: String,
    _subscriber: rosrust::Subscriber,
    _view: Arc<RwLock<Isometry3<f64>>>
}

impl LaserListener {
    pub fn new(
        config: ListenerConfigColor,
        tf_listener: Arc<rustros_tf::TfListener>,
        static_frame: String,
        _view: Arc<RwLock<Isometry3<f64>>>,

    ) -> LaserListener {
        let scan_points = Arc::new(RwLock::new(Vec::<(f64, f64)>::new()));
        let cb_scan_points = scan_points.clone();
        let str_ = static_frame.clone();

        let local_listener = tf_listener.clone();
        let local_v =_view.clone();

        let laser_sub = rosrust::subscribe(
            &config.topic,
            2,
            move |scan: rosrust_msg::sensor_msgs::LaserScan| {
                let mut points: Vec<(f64, f64)> = Vec::new();
                let res = local_listener.lookup_transform(
                    &str_,
                    &scan.header.frame_id,
                    scan.header.stamp,
                );
                match &res {
                    Ok(res) => res,
                    Err(_e) => return,
                };
                let ref_tf = Isometry3::clone(&local_v.clone().read().unwrap());
                let tf = isometry_to_transform(ros_transform_to_isometry(&res.as_ref().unwrap().transform) * ref_tf);

                for (i, range) in scan.ranges.iter().enumerate() {
                    let angle = scan.angle_min + i as f32 * scan.angle_increment;
                    let pt = transformation::transform_relative_pt(
                        &tf,
                        (
                            *range as f64 * angle.cos() as f64,
                            *range as f64 * angle.sin() as f64,
                        ),
                    );
                    if range > &scan.range_min {
                        points.push(pt);
                    }
                }
                let mut cb_scan_points = cb_scan_points.write().unwrap();
                *cb_scan_points = points;
            },
        )
        .unwrap();

        LaserListener {
            config,
            points: scan_points,
            _tf_listener: tf_listener.clone(),
            _static_frame: static_frame.to_string(),
            _subscriber: laser_sub,
            _view: _view.clone(),
        }
    }
}
