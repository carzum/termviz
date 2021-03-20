use std::sync::{Arc, Mutex, RwLock};

use rosrust;
use rosrust_msg;
use rustros_tf;
use nalgebra::geometry::{Quaternion, UnitQuaternion, Isometry3, Translation3};


pub struct InitialPosePub {
    publisher: rosrust::Publisher<rosrust_msg::geometry_msgs::PoseWithCovarianceStamped>,
    _frame_id: String,
    _tf_listener: Arc<Mutex<rustros_tf::TfListener>>,
    ref_transform: Arc<RwLock<rosrust_msg::geometry_msgs::Transform>>,
    static_frame: String,
}

impl InitialPosePub {
    pub fn new(frame_id: &str,
               tf_listener: Arc<Mutex<rustros_tf::TfListener>>,
               ref_transform: Arc<RwLock<rosrust_msg::geometry_msgs::Transform>>,
               static_frame: String,
               ) -> InitialPosePub
    {
        let publisher = rosrust::publish("initialpose", 1).unwrap();
        InitialPosePub{
            publisher: publisher,
            _frame_id: frame_id.to_string(),
            _tf_listener: tf_listener,
            ref_transform: ref_transform,
            static_frame: static_frame,
        }
    }

    pub fn send_estimate(&self, x: f64, y: f64, rotation: f64)
    {
        let mut msg = rosrust_msg::geometry_msgs::PoseWithCovarianceStamped::default();

        // diff
        let tra = Translation3::new(x, y, 0.0);
        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, rotation);
        let isometry = Isometry3::from_parts(tra, q);


        let tf = &self.ref_transform.as_ref().read().unwrap();
        let cur_tra = Translation3::new(tf.translation.x, tf.translation.y,
                                        tf.translation.z);
    // 3: z 2: y, 1: x, 0: w
        let cur_rot = UnitQuaternion::new_normalize(Quaternion::new(
                tf.rotation.w, tf.rotation.x, tf.rotation.y, tf.rotation.z));
        let cur_isometry = Isometry3::from_parts(cur_tra, cur_rot);

        let iso = cur_isometry * isometry;
        msg.header.frame_id = self.static_frame.to_string();
        msg.pose.pose.position.x = 1.0;
        msg.pose.pose.orientation.x = iso.rotation[0];
        msg.pose.pose.orientation.y = iso.rotation[1];
        msg.pose.pose.orientation.z = iso.rotation[2];
        msg.pose.pose.orientation.w = iso.rotation[3];
        msg.pose.pose.position.x = iso.translation.x;
        msg.pose.pose.position.y = iso.translation.y;
        msg.pose.pose.position.z = 0.0;
        self.publisher.send(msg).unwrap();
    }
}
