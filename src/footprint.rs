use crate::transformation;

use rosrust;
use rosrust_msg;

const DEFAULT_FOOTPRINT: [[f64; 2]; 4] =
    [[0.25, 0.25], [-0.25, 0.25], [-0.25, -0.25], [0.25, -0.25]];

pub fn get_default_footprint() -> Vec<(f64, f64)> {
    let mut result = Vec::<(f64, f64)>::new();
    for pt in DEFAULT_FOOTPRINT {
        result.push((pt[0], pt[1]));
    }
    result
}

pub fn get_footprint() -> Vec<(f64, f64)> {
    let param = rosrust::param("/footprint");
    let mut result = Vec::<(f64, f64)>::new();
    match param {
        Some(footprint) => {
            let fb = footprint.get::<Vec<Vec<f64>>>();
            match fb {
                Ok(f) => {
                    for pt in f {
                        result.push((pt[0], pt[1]));
                    }
                    result
                }
                Err(_e) => {
                    println!("/footprint not found, using default footprint.");
                    get_default_footprint()
                }
            }
        }
        None => {
            println!("/footprint not found, using default footprint.");
            get_default_footprint()
        }
    }
}

pub fn get_current_footprint(
    tf: &std::sync::RwLockReadGuard<rosrust_msg::geometry_msgs::Transform>,
    footprint_poly: &Vec<(f64, f64)>,
) -> Vec<(f64, f64, f64, f64)> {
    let mut result: Vec<(f64, f64, f64, f64)> = Vec::new();
    for i in 0..footprint_poly.len() - 1 {
        let pt0 = transformation::transform_relative_pt(&tf, footprint_poly[i]);
        let pt1 = transformation::transform_relative_pt(&tf, footprint_poly[i + 1]);
        result.push((pt0.0, pt0.1, pt1.0, pt1.1));
    }

    // close footprint
    let p_end =
        transformation::transform_relative_pt(&tf, footprint_poly[footprint_poly.len() - 1]);
    let p_start = transformation::transform_relative_pt(&tf, footprint_poly[0]);
    result.push((p_end.0, p_end.1, p_start.0, p_start.1));
    result
}
