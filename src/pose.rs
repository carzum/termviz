use crate::config::{Color, PoseListenerConfig};
use crate::ros;
use crate::ros::types;
use crate::transformation::ros_pose_to_isometry;
use nalgebra::geometry::{Isometry3, Point3};
use std::option::Option;
use std::sync::{Arc, RwLock};
use tui::style;
use tui::widgets::canvas::Line;

fn pose_to_arrow(pose: &Isometry3<f64>, length: f64, color: &Color) -> Vec<Line> {
    let mut lines: Vec<Line> = Vec::new();
    let tui_color = style::Color::Rgb(color.r, color.g, color.b);
    let pt1 = pose.transform_point(&Point3::new(0.0, 0.0, 0.0));
    let pt2 = pose.transform_point(&Point3::new(length, 0.0, 0.0));
    lines.push(Line {
        x1: pt1.x,
        y1: pt1.y,
        x2: pt2.x,
        y2: pt2.y,
        color: tui_color,
    });
    let pt_left = pose.transform_point(&Point3::new(length / 4.0 * 2.0, length / 4.0, 0.0));
    lines.push(Line {
        x1: pt2.x,
        y1: pt2.y,
        x2: pt_left.x,
        y2: pt_left.y,
        color: tui_color,
    });
    let pt_right = pose.transform_point(&Point3::new(length / 4.0 * 2.0, -length / 4.0, 0.0));
    lines.push(Line {
        x1: pt2.x,
        y1: pt2.y,
        x2: pt_right.x,
        y2: pt_right.y,
        color: tui_color,
    });
    lines
}

fn pose_to_axes(pose: &Isometry3<f64>, length: f64) -> Vec<Line> {
    let mut lines: Vec<Line> = Vec::new();
    let origin = pose.transform_point(&Point3::new(0.0, 0.0, 0.0));
    let x_axis = pose.transform_point(&Point3::new(length, 0.0, 0.0));
    let y_axis = pose.transform_point(&Point3::new(0.0, length, 0.0));
    let z_axis = pose.transform_point(&Point3::new(0.0, 0.0, length));
    lines.push(Line {
        x1: origin.x,
        y1: origin.y,
        x2: x_axis.x,
        y2: x_axis.y,
        color: style::Color::Red,
    });
    lines.push(Line {
        x1: origin.x,
        y1: origin.y,
        x2: y_axis.x,
        y2: y_axis.y,
        color: style::Color::Green,
    });
    lines.push(Line {
        x1: origin.x,
        y1: origin.y,
        x2: z_axis.x,
        y2: z_axis.y,
        color: style::Color::Blue,
    });
    lines
}

fn poses_to_lines(poses: &Vec<Isometry3<f64>>, color: &Color) -> Vec<Line> {
    poses
        .windows(2)
        .map(|w| {
            let p0 = w[0].transform_point(&Point3::new(0.0, 0.0, 0.0));
            let p1 = w[1].transform_point(&Point3::new(0.0, 0.0, 0.0));
            Line {
                x1: p0.x,
                y1: p0.y,
                x2: p1.x,
                y2: p1.y,
                color: style::Color::Rgb(color.r, color.g, color.b),
            }
        })
        .collect()
}

pub struct PoseStampedListener {
    config: PoseListenerConfig,
    pose: Arc<RwLock<Option<Isometry3<f64>>>>,
    _subscriber: ros::SubscriptionHandle,
}

impl PoseStampedListener {
    pub fn new(config: PoseListenerConfig) -> PoseStampedListener {
        let pose = Arc::new(RwLock::new(None));
        let cb_pose = pose.clone();
        let sub = ros::subscribe_pose_stamped(&config.topic, 2, move |pose_msg: types::PoseStamped| {
                let pose_iso = ros_pose_to_isometry(&pose_msg.pose);
                *cb_pose.write().unwrap() = Some(pose_iso);
            })
            .unwrap();

        PoseStampedListener {
            config: config,
            pose: pose,
            _subscriber: sub,
        }
    }

    pub fn get_lines(&self) -> Vec<Line> {
        match *self.pose.read().unwrap() {
            Some(p) => match self.config.style.as_str() {
                "arrow" => pose_to_arrow(&p, self.config.length, &self.config.color),
                "axes" => pose_to_axes(&p, self.config.length),
                _ => Vec::new(),
            },
            None => Vec::new(),
        }
    }
}

pub struct PoseArrayListener {
    config: PoseListenerConfig,
    poses: Arc<RwLock<Vec<Isometry3<f64>>>>,
    _subscriber: ros::SubscriptionHandle,
}

impl PoseArrayListener {
    pub fn new(config: PoseListenerConfig) -> PoseArrayListener {
        let poses = Arc::new(RwLock::new(Vec::<Isometry3<f64>>::new()));
        let cb_poses = poses.clone();
        let sub = ros::subscribe_pose_array(&config.topic, 2, move |pose_array: types::PoseArray| {
                let poses_iso = pose_array
                    .poses
                    .into_iter()
                    .map(|p| ros_pose_to_isometry(&p))
                    .collect();
                *cb_poses.write().unwrap() = poses_iso;
            })
            .unwrap();

        PoseArrayListener {
            config: config,
            poses: poses,
            _subscriber: sub,
        }
    }

    pub fn get_lines(&self) -> Vec<Line> {
        if self.poses.read().unwrap().is_empty() {
            return Vec::new();
        }
        match self.config.style.as_str() {
            "arrow" => self
                .poses
                .read()
                .unwrap()
                .iter()
                .map(|p| pose_to_arrow(&p, self.config.length, &self.config.color))
                .collect::<Vec<Vec<Line>>>()
                .into_iter()
                .reduce(|a, b| a.into_iter().chain(b.into_iter()).collect())
                .unwrap(),
            "axis" => self
                .poses
                .read()
                .unwrap()
                .iter()
                .map(|p| pose_to_axes(&p, self.config.length))
                .collect::<Vec<Vec<Line>>>()
                .into_iter()
                .reduce(|a, b| a.into_iter().chain(b.into_iter()).collect())
                .unwrap(),
            _ => Vec::new(),
        }
    }
}

pub struct PathListener {
    config: PoseListenerConfig,
    poses: Arc<RwLock<Vec<Isometry3<f64>>>>,
    _subscriber: ros::SubscriptionHandle,
}

impl PathListener {
    pub fn new(config: PoseListenerConfig) -> PathListener {
        let poses = Arc::new(RwLock::new(Vec::<Isometry3<f64>>::new()));
        let cb_poses = poses.clone();
        let sub = ros::subscribe_path(&config.topic, 2, move |path: types::Path| {
                let poses_iso = path
                    .poses
                    .into_iter()
                    .map(|p| ros_pose_to_isometry(&p.pose))
                    .collect();
                *cb_poses.write().unwrap() = poses_iso;
            })
            .unwrap();

        PathListener {
            config: config,
            poses: poses,
            _subscriber: sub,
        }
    }

    pub fn get_lines(&self) -> Vec<Line> {
        if self.poses.read().unwrap().is_empty() {
            return Vec::new();
        }
        match self.config.style.as_str() {
            "arrow" => self
                .poses
                .read()
                .unwrap()
                .iter()
                .map(|p| pose_to_arrow(&p, self.config.length, &self.config.color))
                .collect::<Vec<Vec<Line>>>()
                .into_iter()
                .reduce(|a, b| a.into_iter().chain(b.into_iter()).collect())
                .unwrap(),
            "axis" => self
                .poses
                .read()
                .unwrap()
                .iter()
                .map(|p| pose_to_axes(&p, self.config.length))
                .collect::<Vec<Vec<Line>>>()
                .into_iter()
                .reduce(|a, b| a.into_iter().chain(b.into_iter()).collect())
                .unwrap(),
            "line" => poses_to_lines(&self.poses.read().unwrap(), &self.config.color),
            _ => Vec::new(),
        }
    }
}
