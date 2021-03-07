use rustros_tf;
use nalgebra::geometry::{Quaternion, UnitQuaternion, Point3, Isometry3, Translation3};


pub fn transform_relative_pt(tf: &std::sync::RwLockReadGuard<rustros_tf::msg::geometry_msgs::Transform>,
                             pt: (f64, f64)) -> (f64, f64) {
    let tra = Translation3::new(tf.translation.x, tf.translation.y,
                                tf.translation.z);
    // 3: z 2: y, 1: x, 0: w
    let rot = UnitQuaternion::new_normalize(Quaternion::new(
            tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w));
    let isometry = Isometry3::from_parts(tra, rot);
    let transformed_point = isometry.transform_point(&Point3::new(pt.0, pt.1, 0.));
    (transformed_point.x, transformed_point.y)
}

pub fn transform_relative_point(tf: &rustros_tf::msg::geometry_msgs::Transform,
                                pt: (f64, f64)) -> (f64, f64) {
    let tra = Translation3::new(tf.translation.x, tf.translation.y,
                                tf.translation.z);
    // 3: z 2: y, 1: x, 0: w
    let rot = UnitQuaternion::new_normalize(Quaternion::new(
            tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w));
    let isometry = Isometry3::from_parts(tra, rot);
    let transformed_point = isometry.transform_point(&Point3::new(pt.0, pt.1, 0.));
    (transformed_point.x, transformed_point.y)
}
