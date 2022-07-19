use byteorder::{ByteOrder, LittleEndian};
use crate::config::PointCloud2ListenerConfig;
use std::sync::{Arc, RwLock};

use nalgebra::geometry::Point3;
use tui::style::Color;

use rosrust;
use rustros_tf;
use crate::transformation::ros_transform_to_isometry;

pub struct PointCloud2Listener {
    pub config: PointCloud2ListenerConfig,
    pub points: Arc<RwLock<Vec<(f64, f64)>>>,
    pub colors: Arc<RwLock<Vec<Color>>>,
    _tf_listener: Arc<rustros_tf::TfListener>,
    _static_frame: String,
    _subscriber: rosrust::Subscriber,
}

pub fn get_channel_offset(name: &str, fields: &Vec<rosrust_msg::sensor_msgs::PointField>) -> u32
{
    for field in fields {
        if field.name == name {
            return field.offset
        }
    }
    panic!("Could not find field {:}", name);
}

pub fn read_f32(bytes: &Vec<u8>, idx: u32) -> f32
{
    LittleEndian::read_f32(&bytes[idx as usize..(idx+ 4) as usize])
}

pub fn read_xyz(msg: &rosrust_msg::sensor_msgs::PointCloud2) -> Vec<Point3<f64>>
{
    let n_pts = msg.width * msg.height;
    let mut points: Vec<Point3<f64>> = Vec::with_capacity(n_pts as usize);
    let x_offset = get_channel_offset("x", &msg.fields);
    let y_offset = get_channel_offset("y", &msg.fields);
    let z_offset = get_channel_offset("z", &msg.fields);
    for i in 0..n_pts {
        let idx = i * msg.point_step;
        /* let x = LittleEndian::read_f32(&msg.data[(i + x_offset) as usize..(i + x_offset + 4) as usize]);
        let y = LittleEndian::read_f32(&msg.data[(i + y_offset) as usize..(i + y_offset + 4) as usize]);
        let z = LittleEndian::read_f32(&msg.data[(i + z_offset) as usize..(i + z_offset + 4) as usize]); */
        points.push(Point3::new(read_f32(&msg.data, idx + x_offset) as f64,
                                read_f32(&msg.data, idx + y_offset) as f64,
                                read_f32(&msg.data, idx + z_offset) as f64));
    }
    points
}

pub fn read_rgb(msg: &rosrust_msg::sensor_msgs::PointCloud2) -> Vec<Color>
{
    let n_pts = msg.width * msg.height;
    let mut colors: Vec<Color> = Vec::with_capacity(n_pts as usize);
    let rgb_offset = get_channel_offset("rgb", &msg.fields);
    for i in 0..n_pts {
        let idx = i * msg.point_step + rgb_offset;
        colors.push(Color::Rgb(msg.data[(idx + 2) as usize],
                               msg.data[(idx + 1) as usize],
                               msg.data[idx as usize]));

    }
    colors
}

/* pub fn read_pointfield(
    data: Vec<u8>,
    offset: u32,
    count: u32,
    is_bigendian: bool) -> bool
{
    f32.
    for i in 0..count {
        data[i + offset
    }
    // iterate through data with
    //msg.fields
    //msg.

    /* # This message holds the description of one point entry in the
    # PointCloud2 message format.
    uint8 INT8    = 1
    uint8 UINT8   = 2
    uint8 INT16   = 3
    uint8 UINT16  = 4
    uint8 INT32   = 5
    uint8 UINT32  = 6
    uint8 FLOAT32 = 7
    uint8 FLOAT64 = 8

    string name      # Name of field
    uint32 offset    # Offset from start of point struct
    uint8  datatype  # Datatype enumeration, see above
    uint32 count     # How many elements in the field */
    true
} */

impl PointCloud2Listener {
    pub fn new(
        config: PointCloud2ListenerConfig,
        tf_listener: Arc<rustros_tf::TfListener>,
        static_frame: String,
    ) -> PointCloud2Listener {
        let occ_points = Arc::new(RwLock::new(Vec::<(f64, f64)>::new()));
        let cb_occ_points = occ_points.clone();
        let colors = Arc::new(RwLock::new(Vec::<Color>::new()));
        let cb_colors = colors.clone();
        let str_ = static_frame.clone();
        let local_listener = tf_listener.clone();
        let _sub = rosrust::subscribe(
            &config.topic,
            1,
            move |cloud: rosrust_msg::sensor_msgs::PointCloud2| {
                let mut points: Vec<(f64, f64)> = Vec::new();
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
                for pt in read_xyz(&cloud) {
                    let trans_pt = isometry.transform_point(&pt);
                    points.push((trans_pt.x, trans_pt.y));
                }
                let mut cb_occ_points = cb_occ_points.write().unwrap();
                *cb_occ_points = points;
                let mut cb_colors = cb_colors.write().unwrap();
                *cb_colors = read_rgb(&cloud);

            },
        )
        .unwrap();

        PointCloud2Listener {
            config,
            points: occ_points,
            colors: colors,
            _tf_listener: tf_listener,
            _static_frame: static_frame.to_string(),
            _subscriber: _sub,
        }
    }
}
