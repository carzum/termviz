use std::sync::Arc;

use nalgebra::geometry::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use rosrust;
use rosrust_msg;

pub struct InitialPosePub {
    publisher: rosrust::Publisher<rosrust_msg::geometry_msgs::PoseWithCovarianceStamped>,
    _frame_id: String,
    static_frame: String,
    tf_listener: Arc<rustros_tf::TfListener>,
}

impl InitialPosePub {
    pub fn new(
        frame_id: &str,
        static_frame: String,
        tf_listener: Arc<rustros_tf::TfListener>,
    ) -> InitialPosePub {
        let publisher = rosrust::publish("initialpose", 1).unwrap();
        InitialPosePub {
            publisher: publisher,
            _frame_id: frame_id.to_string(),
            static_frame: static_frame,
            tf_listener: tf_listener,
        }
    }

    pub fn send_estimate(&self, x: f64, y: f64, rotation: f64) {
        let mut msg = rosrust_msg::geometry_msgs::PoseWithCovarianceStamped::default();

        // diff
        let tra = Translation3::new(x, y, 0.0);
        let q = UnitQuaternion::from_euler_angles(0.0, 0.0, rotation);
        let isometry = Isometry3::from_parts(tra, q);

        let tf = &self
            .tf_listener
            .lookup_transform(&self.static_frame, &self._frame_id, rosrust::Time::new())
            .unwrap();
        let cur_tra = Translation3::new(
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        );
        // 3: z 2: y, 1: x, 0: w
        let cur_rot = UnitQuaternion::new_normalize(Quaternion::new(
            tf.transform.rotation.w,
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
        ));
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
