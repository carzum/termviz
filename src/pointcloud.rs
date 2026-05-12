use crate::config::PointCloud2ListenerConfig;
use crate::ros;
use crate::ros::tf::TfClient;
use crate::ros::types;
use byteorder::{ByteOrder, LittleEndian};
use colorgrad::Gradient;
use std::sync::{Arc, RwLock};

use nalgebra::geometry::Point3;
use tui::style::Color;

use crate::transformation::ros_transform_to_isometry;

pub struct PointCloud2Listener {
    pub points: Arc<RwLock<Vec<ColoredPoint>>>,
    _tf: Arc<dyn TfClient>,
    _static_frame: String,
    _subscriber: ros::SubscriptionHandle,
}

#[derive(Clone)]
pub struct ColoredPoint {
    pub point: Point3<f64>,
    pub color: Color,
}

impl ColoredPoint {
    pub fn new(point: Option<Point3<f64>>, color: Option<Color>) -> ColoredPoint {
        ColoredPoint {
            point: point.unwrap_or(Point3::new(0.0, 0.0, 0.0)),
            color: color.unwrap_or(Color::Red),
        }
    }
}

pub fn get_channel_offset(name: &str, fields: &Vec<types::PointField>) -> u32 {
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

pub fn read_xyz(msg: &types::PointCloud2) -> Vec<Point3<f64>> {
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

pub fn colorize_from_rgb(
    mut points: Vec<ColoredPoint>,
    msg: &types::PointCloud2,
) -> Vec<ColoredPoint> {
    let n_pts = msg.width * msg.height;
    let rgb_offset = get_channel_offset("rgb", &msg.fields);
    for i in 0..n_pts {
        let idx = i * msg.point_step + rgb_offset;
        points[i as usize].color = Color::Rgb(
            msg.data[(idx + 2) as usize],
            msg.data[(idx + 1) as usize],
            msg.data[idx as usize],
        );
    }
    points
}

pub fn colorize_points(mut points: Vec<ColoredPoint>, min_z: f64, max_z: f64) -> Vec<ColoredPoint> {
    let grad = colorgrad::preset::turbo();
    for pt in points.iter_mut() {
        let c = grad
            .at((pt.point.z - min_z) as f32 / (max_z - min_z) as f32)
            .to_rgba8();
        pt.color = Color::Rgb(c[0], c[1], c[2]);
    }
    points
}

impl PointCloud2Listener {
    pub fn new(
        config: PointCloud2ListenerConfig,
        tf: Arc<dyn TfClient>,
        static_frame: String,
    ) -> PointCloud2Listener {
        let occ_points = Arc::new(RwLock::new(Vec::<ColoredPoint>::new()));
        let cb_occ_points = occ_points.clone();
        let str_ = static_frame.clone();
        let local_tf = tf.clone();
        let use_rgb = config.use_rgb.clone();
        let _sub = ros::subscribe_pointcloud2(&config.topic, 1, move |cloud: types::PointCloud2| {
                let mut points: Vec<ColoredPoint> = Vec::new();
                let res = local_tf.lookup_transform(&str_, &cloud.header.frame_id, cloud.header.stamp);
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
                    points.push(ColoredPoint::new(Some(trans_pt), None));
                }
                if use_rgb {
                    points = colorize_from_rgb(points, &cloud);
                } else {
                    points = colorize_points(points, min_z, max_z);
                }
                points = points
                    .into_iter()
                    .filter(|n| !n.point.z.is_nan())
                    .collect::<Vec<_>>();
                let mut cb_occ_points = cb_occ_points.write().unwrap();
                *cb_occ_points = points;
            })
            .unwrap();

        PointCloud2Listener {
            points: occ_points,
            _tf: tf,
            _static_frame: static_frame.to_string(),
            _subscriber: _sub,
        }
    }
}
