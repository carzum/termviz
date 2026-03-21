#![allow(dead_code)]

use std::time::Duration as StdDuration;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Time {
    pub sec: i64,
    pub nsec: u32,
}

impl Time {
    #[allow(dead_code)]
    pub const fn zero() -> Self {
        Self { sec: 0, nsec: 0 }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Duration {
    pub sec: i64,
    pub nsec: u32,
}

impl Duration {
    #[allow(dead_code)]
    pub fn to_std(self) -> Option<StdDuration> {
        if self.sec < 0 {
            return None;
        }
        Some(StdDuration::new(self.sec as u64, self.nsec))
    }
}

#[derive(Clone, Debug, Default)]
pub struct Header {
    pub frame_id: String,
    pub stamp: Time,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Point {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Transform {
    pub translation: Vector3,
    pub rotation: Quaternion,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Pose {
    pub position: Point,
    pub orientation: Quaternion,
}

#[derive(Clone, Debug, Default)]
pub struct TransformStamped {
    pub parent_frame: String,
    pub child_frame: String,
    pub stamp: Time,
    pub transform: Transform,
}

#[derive(Clone, Debug, Default)]
pub struct LaserScan {
    pub header: Header,
    pub angle_min: f32,
    pub angle_increment: f32,
    pub ranges: Vec<f32>,
    pub range_min: f32,
}

#[derive(Clone, Debug, Default)]
pub struct MapMetaData {
    pub resolution: f32,
    pub width: u32,
    pub height: u32,
    pub origin: Pose,
}

#[derive(Clone, Debug, Default)]
pub struct OccupancyGrid {
    pub header: Header,
    pub info: MapMetaData,
    pub data: Vec<i8>,
}

#[derive(Clone, Debug, Default)]
pub struct PointField {
    pub name: String,
    pub offset: u32,
}

#[derive(Clone, Debug, Default)]
pub struct PointCloud2 {
    pub header: Header,
    pub width: u32,
    pub height: u32,
    pub fields: Vec<PointField>,
    pub point_step: u32,
    pub is_bigendian: bool,
    pub data: Vec<u8>,
}

#[derive(Clone, Debug, Default)]
pub struct Polygon {
    pub points: Vec<Point>,
}

#[derive(Clone, Debug, Default)]
pub struct PolygonStamped {
    pub header: Header,
    pub polygon: Polygon,
}

#[derive(Clone, Debug, Default)]
pub struct Image {
    pub header: Header,
    pub width: u32,
    pub height: u32,
    pub encoding: String,
    pub is_bigendian: bool,
    pub data: Vec<u8>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ColorRGBA {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

#[derive(Clone, Debug, Default)]
pub struct Marker {
    pub header: Header,
    pub ns: String,
    pub id: i32,
    pub type_: i32,
    pub action: i32,
    pub pose: Pose,
    pub scale: Vector3,
    pub color: ColorRGBA,
    pub points: Vec<Point>,
    pub colors: Vec<ColorRGBA>,
    pub lifetime: Duration,
}

#[allow(dead_code)]
impl Marker {
    // visualization_msgs/Marker action constants
    pub const ADD: i32 = 0;
    pub const DELETE: i32 = 2;
    pub const DELETEALL: i32 = 3;

    // visualization_msgs/Marker type constants
    pub const ARROW: i32 = 0;
    pub const CUBE: i32 = 1;
    pub const SPHERE: i32 = 2;
    pub const CYLINDER: i32 = 3;
    pub const LINE_STRIP: i32 = 4;
    pub const LINE_LIST: i32 = 5;
    pub const CUBE_LIST: i32 = 6;
    pub const SPHERE_LIST: i32 = 7;
    pub const POINTS: i32 = 8;
    pub const TEXT_VIEW_FACING: i32 = 9;
    pub const MESH_RESOURCE: i32 = 10;
    pub const TRIANGLE_LIST: i32 = 11;
}

#[derive(Clone, Debug, Default)]
pub struct MarkerArray {
    pub markers: Vec<Marker>,
}

#[derive(Clone, Debug, Default)]
pub struct PoseStamped {
    pub header: Header,
    pub pose: Pose,
}

#[derive(Clone, Debug, Default)]
pub struct PoseArray {
    pub header: Header,
    pub poses: Vec<Pose>,
}

#[derive(Clone, Debug, Default)]
pub struct Path {
    pub header: Header,
    pub poses: Vec<PoseStamped>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Twist {
    pub linear_x: f64,
    pub linear_y: f64,
    pub angular_z: f64,
}
