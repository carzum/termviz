use nalgebra::geometry::{Isometry2, Isometry3, Point3, Quaternion, Translation3, UnitQuaternion};
use nalgebra::Vector2;

use crate::ros::types;

pub fn transform_relative_pt(
    tf: &types::Transform,
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

pub fn ros_to_iso2d(tf: &types::Transform) -> Isometry2<f64> {
    let rot = UnitQuaternion::new_normalize(Quaternion::new(
        tf.rotation.w,
        tf.rotation.x,
        tf.rotation.y,
        tf.rotation.z,
    ));
    let (_roll, _pitch, yaw) = rot.euler_angles();
    Isometry2::new(Vector2::new(tf.translation.x, tf.translation.y), yaw)
}

pub fn ros_pose_to_isometry(pose: &types::Pose) -> Isometry3<f64> {
    let tra = Translation3::new(pose.position.x, pose.position.y, pose.position.z);
    let rot = UnitQuaternion::new_normalize(Quaternion::new(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
    ));
    Isometry3::from_parts(tra, rot)
}

pub fn iso2d_to_ros(tf: &Isometry2<f64>) -> types::Transform {
    let rot = UnitQuaternion::from_euler_angles(0.0, 0.0, tf.rotation.angle());
    types::Transform {
        translation: types::Vector3 {
            x: tf.translation.x,
            y: tf.translation.y,
            z: 0.0,
        },
        rotation: types::Quaternion {
            x: rot.quaternion()[0],
            y: rot.quaternion()[1],
            z: rot.quaternion()[2],
            w: rot.quaternion()[3],
        },
    }
}

pub fn ros_transform_to_isometry(tf: &types::Transform) -> Isometry3<f64> {
    let tra = Translation3::new(tf.translation.x, tf.translation.y, tf.translation.z);
    let rot = UnitQuaternion::new_normalize(Quaternion::new(
        tf.rotation.w,
        tf.rotation.x,
        tf.rotation.y,
        tf.rotation.z,
    ));
    Isometry3::from_parts(tra, rot)
}
