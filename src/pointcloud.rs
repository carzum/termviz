use crate::config::PointCloud2ListenerConfig;
use byteorder::{ByteOrder, LittleEndian};
use colorgrad;
use std::sync::{Arc, RwLock};

use nalgebra::geometry::Point3;
use tui::style::Color;

use crate::transformation::ros_transform_to_isometry;
use rosrust;
use rustros_tf;

pub struct PointCloud2Listener {
    pub config: PointCloud2ListenerConfig,
    pub points: Arc<RwLock<Vec<Point3<f64>>>>,
    pub colors: Arc<RwLock<Vec<Color>>>,
    _tf_listener: Arc<rustros_tf::TfListener>,
    _static_frame: String,
    _subscriber: rosrust::Subscriber,
}

pub fn get_channel_offset(name: &str, fields: &Vec<rosrust_msg::sensor_msgs::PointField>) -> u32 {
    for field in fields {
        if field.name == name {
            return field.offset;
        }
    }
    panic!("Could not find field {:}", name);
}

pub fn read_f32(bytes: &Vec<u8>, idx: u32) -> f32 {
    LittleEndian::read_f32(&bytes[idx as usize..(idx + 4) as usize])
}

pub fn read_xyz(msg: &rosrust_msg::sensor_msgs::PointCloud2) -> Vec<Point3<f64>> {
    let n_pts = msg.width * msg.height;
    let mut points: Vec<Point3<f64>> = Vec::with_capacity(n_pts as usize);
    let x_offset = get_channel_offset("x", &msg.fields);
    let y_offset = get_channel_offset("y", &msg.fields);
    let z_offset = get_channel_offset("z", &msg.fields);
    for i in 0..n_pts {
        let idx = i * msg.point_step;
        points.push(Point3::new(
            read_f32(&msg.data, idx + x_offset) as f64,
            read_f32(&msg.data, idx + y_offset) as f64,
            read_f32(&msg.data, idx + z_offset) as f64,
        ));
    }
    points
}

pub fn read_rgb(msg: &rosrust_msg::sensor_msgs::PointCloud2) -> Vec<Color> {
    let n_pts = msg.width * msg.height;
    let mut colors: Vec<Color> = Vec::with_capacity(n_pts as usize);
    let rgb_offset = get_channel_offset("rgb", &msg.fields);
    for i in 0..n_pts {
        let idx = i * msg.point_step + rgb_offset;
        colors.push(Color::Rgb(
            msg.data[(idx + 2) as usize],
            msg.data[(idx + 1) as usize],
            msg.data[idx as usize],
        ));
    }
    colors
}

pub fn colorize_points(points: &Vec<Point3<f64>>, min_z: f64, max_z: f64) -> Vec<Color> {
    let mut colors: Vec<Color> = Vec::with_capacity(points.len());
    let grad = colorgrad::turbo();
    for pt in points {
        let c = grad.at((pt.z - min_z) / (max_z - min_z)).to_rgba8();
        colors.push(Color::Rgb(c[0], c[1], c[2]));
    }
    colors
}

impl PointCloud2Listener {
    pub fn new(
        config: PointCloud2ListenerConfig,
        tf_listener: Arc<rustros_tf::TfListener>,
        static_frame: String,
    ) -> PointCloud2Listener {
        let occ_points = Arc::new(RwLock::new(Vec::<Point3<f64>>::new()));
        let cb_occ_points = occ_points.clone();
        let colors = Arc::new(RwLock::new(Vec::<Color>::new()));
        let cb_colors = colors.clone();
        let str_ = static_frame.clone();
        let local_listener = tf_listener.clone();
        let _sub = rosrust::subscribe(
            &config.topic,
            1,
            move |cloud: rosrust_msg::sensor_msgs::PointCloud2| {
                let mut points: Vec<Point3<f64>> = Vec::new();
                let res = local_listener.clone().lookup_transform(
                    &str_,
                    &cloud.header.frame_id,
                    cloud.header.stamp,
                );
                match &res {
                    Ok(res) => res,
                    Err(_e) => return,
                };

                let isometry = ros_transform_to_isometry(&res.unwrap().transform);
                let mut max_z = f64::MIN;
                let mut min_z = f64::MAX;
                for pt in read_xyz(&cloud) {
                    let trans_pt = isometry.transform_point(&pt);
                    if trans_pt.z > max_z {
                        max_z = trans_pt.z;
                    }
                    if trans_pt.z < min_z {
                        min_z = trans_pt.z;
                    }
                    points.push(trans_pt);
                }
                let mut cb_colors = cb_colors.write().unwrap();
                if false {
                    *cb_colors = read_rgb(&cloud);
                } else {
                    *cb_colors = colorize_points(&points, min_z, max_z);
                }
                let mut cb_occ_points = cb_occ_points.write().unwrap();
                *cb_occ_points = points;
            },
        )
        .unwrap();

        PointCloud2Listener {
            config,
            points: occ_points,
            colors,
            _tf_listener: tf_listener,
            _static_frame: static_frame.to_string(),
            _subscriber: _sub,
        }
    }
}
