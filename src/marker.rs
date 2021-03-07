use std::sync::{Arc, Mutex, RwLock};

use rosrust;
use rustros_tf;
use nalgebra::geometry::{Quaternion, UnitQuaternion, Isometry3,Point3, Translation3};
use tui::style::Color;
use tui::widgets::canvas::{Line};


pub fn marker_2_rectancle(msg: &rosrust_msg::visualization_msgs::Marker,
                          tf: &rustros_tf::msg::geometry_msgs::Transform) -> Vec<Line> {
    let mut lines: Vec<Line> = Vec::new();
    // get corner points of box
    let tras = Translation3::new(tf.translation.x, tf.translation.y,
                                 tf.translation.z);
    let rots = UnitQuaternion::new_normalize(Quaternion::new(
            tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z));

    let isometry = Isometry3::from_parts(tras, rots);
    let tra = Translation3::new(msg.pose.position.x,
                                msg.pose.position.y,
                                msg.pose.position.z);

    let rot = UnitQuaternion::new_normalize(Quaternion::new(
            msg.pose.orientation.w,
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z));

    let iso = isometry * Isometry3::from_parts(tra, rot).inverse();
    let color = Color::Rgb((msg.color.r * 255.0) as u8,
                           (msg.color.g * 255.0) as u8,
                           (msg.color.b * 255.0) as u8);

    let p1 = iso.transform_point(&Point3::new(msg.scale.x / 2.,
                                              msg.scale.y / 2., 0.0));
    let p2 = iso.transform_point(&Point3::new(msg.scale.x / 2.,
                                              - msg.scale.y / 2., 0.0));
    let p3 = iso.transform_point(&Point3::new(- msg.scale.x / 2.,
                                              - msg.scale.y / 2., 0.0));
    let p4 = iso.transform_point(&Point3::new(- msg.scale.x / 2.,
                                              msg.scale.y / 2., 0.0));
    lines.push(Line{x1: p1.x, y1: p1.y, x2: p2.x, y2: p2.y, color: color});
    lines.push(Line{x1: p2.x, y1: p2.y, x2: p3.x, y2: p3.y, color: color});
    lines.push(Line{x1: p3.x, y1: p3.y, x2: p4.x, y2: p4.y, color: color});
    lines.push(Line{x1: p4.x, y1: p4.y, x2: p1.x, y2: p1.y, color: color});
    lines
}


pub struct MarkerListener {
    _topic: String,
    _tf_listener: Arc<Mutex<rustros_tf::TfListener>>,
    _static_frame: String,
    _subscriber: rosrust::Subscriber,
    pub lines: Arc<RwLock<Vec<Line>>>,
}

impl MarkerListener {
    pub fn new(topic: &str,
           tf_listener: Arc<Mutex<rustros_tf::TfListener>>,
           static_frame: String
           ) -> MarkerListener
    {
        let blines = Arc::new(RwLock::new(Vec::<Line>::new()));
        let loop_lines = blines.clone();

        let marker_listener = tf_listener.clone();
        let moved_static = static_frame.clone();
        let sub = rosrust::subscribe(topic, 2, move |msg: rosrust_msg::visualization_msgs::MarkerArray| {
            let listener = marker_listener.lock().unwrap();
            for marker in msg.markers {
                if marker.action != 0 { continue }
                let res = &listener.lookup_transform(
                    &moved_static, &marker.header.frame_id, marker.header.stamp);
                match res {
                    Ok(res) => res,
                    Err(_e) => continue,
                };

                let tf = &res.as_ref().unwrap().transform;

                let mut _lines = loop_lines.write().unwrap();
                _lines.extend(marker_2_rectancle(&marker, tf));

            }
            }).unwrap();

        MarkerListener{
            _topic: topic.to_string(),
            _tf_listener: tf_listener,
            _static_frame: static_frame.to_string(),
            _subscriber: sub,
            lines: blines,
        }
    }

    pub fn get_lines(&self) -> Vec<Line>{
        self.lines.read().unwrap().to_vec()
    }
}
