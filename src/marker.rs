use crate::config::ListenerConfig;
use std::sync::{Arc, RwLock};

use rosrust;
use rustros_tf::transforms::nalgebra::geometry::Point3;
use rustros_tf::transforms::{isometry_from_pose, isometry_from_transform};

use tui::style::Color;
use tui::widgets::canvas::Line;

pub fn marker_2_rectancle(
    msg: &rosrust_msg::visualization_msgs::Marker,
    tf: &rosrust_msg::geometry_msgs::Transform,
) -> Vec<Line> {
    let mut lines: Vec<Line> = Vec::new();
    // get corner points of box
    let trans_marker_to_static_frame = isometry_from_transform(tf);
    let trans_to_marker = isometry_from_pose(&msg.pose);

    let iso = trans_marker_to_static_frame.inverse() * trans_to_marker;

    let color = Color::Rgb(
        (msg.color.r * 255.0) as u8,
        (msg.color.g * 255.0) as u8,
        (msg.color.b * 255.0) as u8,
    );

    let p1 = iso.transform_point(&Point3::new(msg.scale.x / 2., msg.scale.y / 2., 0.0));
    let p2 = iso.transform_point(&Point3::new(msg.scale.x / 2., -msg.scale.y / 2., 0.0));
    let p3 = iso.transform_point(&Point3::new(-msg.scale.x / 2., -msg.scale.y / 2., 0.0));
    let p4 = iso.transform_point(&Point3::new(-msg.scale.x / 2., msg.scale.y / 2., 0.0));
    lines.push(Line {
        x1: p1.x,
        y1: p1.y,
        x2: p2.x,
        y2: p2.y,
        color: color,
    });
    lines.push(Line {
        x1: p2.x,
        y1: p2.y,
        x2: p3.x,
        y2: p3.y,
        color: color,
    });
    lines.push(Line {
        x1: p3.x,
        y1: p3.y,
        x2: p4.x,
        y2: p4.y,
        color: color,
    });
    lines.push(Line {
        x1: p4.x,
        y1: p4.y,
        x2: p1.x,
        y2: p1.y,
        color: color,
    });
    lines
}

pub struct MarkerListener {
    pub config: ListenerConfig,
    pub lines: Arc<RwLock<Vec<Line>>>,
    _tf_listener: Arc<rustros_tf::TfListener>,
    _subscriber: rosrust::Subscriber,
}

impl MarkerListener {
    pub fn new(
        config: ListenerConfig,
        tf_listener: Arc<rustros_tf::TfListener>,
        static_frame: String,
    ) -> MarkerListener {
        let blines = Arc::new(RwLock::new(Vec::<Line>::new()));
        let loop_lines = blines.clone();
        let local_listener = tf_listener.clone();

        let _sub = rosrust::subscribe(
            &config.topic,
            2,
            move |msg: rosrust_msg::visualization_msgs::MarkerArray| {
                for marker in msg.markers {
                    if marker.action != 0 {
                        continue;
                    }
                    let res = &local_listener.clone().lookup_transform(
                        &marker.header.frame_id,
                        &static_frame.clone(),
                        marker.header.stamp,
                    );
                    match &res {
                        Ok(res) => res,
                        Err(_e) => continue,
                    };

                    let mut _lines = loop_lines.write().unwrap();
                    _lines.extend(marker_2_rectancle(
                        &marker,
                        &res.as_ref().unwrap().transform,
                    ));
                }
            },
        )
        .unwrap();

        MarkerListener {
            config,
            lines: blines,
            _tf_listener: tf_listener,
            _subscriber: _sub,
        }
    }

    pub fn get_lines(&self) -> Vec<Line> {
        self.lines.read().unwrap().to_vec()
    }
}
