use rosrust;
use rosrust_msg;

pub struct InitialPosePub {
    publisher: rosrust::Publisher<rosrust_msg::geometry_msgs::PoseWithCovarianceStamped>,
    _frame_id: String,
    static_frame: String,
}

impl InitialPosePub {
    pub fn new(frame_id: &str, static_frame: String) -> InitialPosePub {
        let publisher = rosrust::publish("initialpose", 1).unwrap();
        InitialPosePub {
            publisher: publisher,
            _frame_id: frame_id.to_string(),
            static_frame: static_frame,
        }
    }

    pub fn send_estimate(&self, pose: &rosrust_msg::geometry_msgs::Transform) {
        let mut msg = rosrust_msg::geometry_msgs::PoseWithCovarianceStamped::default();
        msg.header.frame_id = self.static_frame.to_string();
        msg.pose.pose.position.x = 1.0;
        msg.pose.pose.orientation.x = pose.rotation.x;
        msg.pose.pose.orientation.y = pose.rotation.y;
        msg.pose.pose.orientation.z = pose.rotation.z;
        msg.pose.pose.orientation.w = pose.rotation.w;
        msg.pose.pose.position.x = pose.translation.x;
        msg.pose.pose.position.y = pose.translation.y;
        msg.pose.pose.position.z = 0.0;
        self.publisher.send(msg).unwrap();
    }
}
