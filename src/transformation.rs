use nalgebra::geometry::{Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};

pub fn transform_relative_pt(
    tf: &rosrust_msg::geometry_msgs::Transform,
    pt: (f64, f64),
) -> (f64, f64) {
    let tra = Translation3::new(tf.translation.x, tf.translation.y, tf.translation.z);
    let rot = UnitQuaternion::new_normalize(Quaternion::new(
        tf.rotation.w,
        tf.rotation.x,
        tf.rotation.y,
        tf.rotation.z,
    ));
    let isometry = Isometry3::from_parts(tra, rot);
    let transformed_point = isometry.transform_point(&Point3::new(pt.0, pt.1, 0.));
    (transformed_point.x, transformed_point.y)
}

pub fn ros_to_minimal(tf: &rosrust_msg::geometry_msgs::Transform) -> (f64, f64, f64) {
    let rot = UnitQuaternion::new_normalize(Quaternion::new(
        tf.rotation.w,
        tf.rotation.x,
        tf.rotation.y,
        tf.rotation.z,
    ));
    let (_roll, _pitch, yaw) = rot.euler_angles();
    (tf.translation.x, tf.translation.y, yaw)
}

pub fn minimal_to_ros(x: f64, y: f64, yaw: f64) -> rosrust_msg::geometry_msgs::Transform {
    let rot = UnitQuaternion::from_euler_angles(0.0, 0.0, yaw);
    rosrust_msg::geometry_msgs::Transform {
        translation: rosrust_msg::geometry_msgs::Vector3 { x: x, y: y, z: 0.0 },
        rotation: rosrust_msg::geometry_msgs::Quaternion {
            x: rot.quaternion()[0],
            y: rot.quaternion()[1],
            z: rot.quaternion()[2],
            w: rot.quaternion()[3],
        },
    }
}
