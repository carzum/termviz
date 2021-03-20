use crate::transformation;

use rosrust;
use rosrust_msg;


pub fn get_footprint() -> Vec<(f64, f64)> {
    let footprint = rosrust::param("/footprint").unwrap();
    let mut result = Vec::<(f64, f64)>::new();
    if footprint.exists().unwrap() {
        let fb = footprint.get::<Vec<Vec<f64>>>().unwrap();
        for pt in fb {
            result.push((pt[0], pt[1]));
        };
    } else {
        println!("could not load footprint");
    }
    result
}

pub fn get_current_footprint(tf: &std::sync::RwLockReadGuard<rosrust_msg::geometry_msgs::Transform>,
                             footprint_poly: &Vec<(f64, f64)>) -> Vec<(f64, f64, f64, f64)> {
    let mut result: Vec<(f64, f64, f64, f64)> = Vec::new();
    for i in 0..footprint_poly.len() - 1{
        let pt0 = transformation::transform_relative_pt(&tf, footprint_poly[i]);
        let pt1 = transformation::transform_relative_pt(&tf, footprint_poly[i + 1]);
        result.push((pt0.0, pt0.1, pt1.0, pt1.1));
        };

    // close footprint
    let p_end = transformation::transform_relative_pt(
        &tf, footprint_poly[footprint_poly.len() - 1]);
    let p_start = transformation::transform_relative_pt(&tf,
                                                        footprint_poly[0]);
    result.push((p_end.0, p_end.1, p_start.0, p_start.1));
    result
}
