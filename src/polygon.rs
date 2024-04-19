use crate::{
    config::ListenerConfigColor, transformation::ros_transform_to_isometry,
};
use nalgebra::Point3;
use rustros_tf;
use std::sync::{Arc, RwLock};
use tui::widgets::canvas::Line;
use tui::style::Color;

pub fn read_points(msg: &rosrust_msg::geometry_msgs::Polygon) -> Vec<Point3<f64>> {
    let n_pts = msg.points.len();
    let mut points: Vec<Point3<f64>> = Vec::with_capacity(n_pts as usize);
    for pt in msg.points.iter() {
        points.push(Point3::new(pt.x as f64, pt.y as f64, pt.z as f64));
    }
    return points;
}

pub struct PolygonData {
    pub polygon_stamped_msg: Option<rosrust_msg::geometry_msgs::PolygonStamped>,
    pub lines_in_static_frame: Option<Vec<Line>>,
    _color: Color,
    _tf_listener: Arc<rustros_tf::TfListener>,
    _static_frame: String,
}

pub struct PolygonListener {
    _data: Arc<RwLock<PolygonData>>,
    _subscriber: rosrust::Subscriber,
}

impl PolygonData {
    pub fn update(&mut self) {
        self.lines_in_static_frame = None;
        if let Some(polygon) = &self.polygon_stamped_msg {
            let transform = self._tf_listener.clone().lookup_transform(
                &self._static_frame.clone(),
                &polygon.header.frame_id,
                polygon.header.stamp,
            );

            match &transform {
                Ok(transform) => {
                    let mut lines: Vec<Line> = Vec::new();
                    let polygon_transform = ros_transform_to_isometry(&transform.transform);
                    let mut previous_point: Option<Point3<f64>> = None;
                    for pt in read_points(&polygon.polygon) {
                        let point_in_static_frame = polygon_transform.transform_point(&pt);
                        if let Some(pp) = previous_point {
                            lines.push(Line {
                                x1: pp.x,
                                y1: pp.y,
                                x2: point_in_static_frame.x,
                                y2: point_in_static_frame.y,
                                color: self._color.clone(),
                            });
                        }
                        previous_point = Some(point_in_static_frame);
                    }

                    self.lines_in_static_frame = Some(lines);
                }
                Err(_e) => {}
            };
        }
    }

    pub fn get_lines(&self) -> Vec<Line> {
        if let Some(lines) = &self.lines_in_static_frame {
            return lines.to_vec();
        }
        return vec![];
    }
}
impl PolygonListener {
    pub fn new(
        config: ListenerConfigColor,
        tf_listener: Arc<rustros_tf::TfListener>,
        static_frame: String,
    ) -> PolygonListener {
        let data = Arc::new(RwLock::new(PolygonData {
            polygon_stamped_msg: None,
            lines_in_static_frame: None,
            _tf_listener: tf_listener,
            _static_frame: static_frame,
            _color: config.color.to_tui(),
        }));

        let cloned_data = data.clone();
        let sub = rosrust::subscribe(
            &config.topic,
            1,
            move |msg: rosrust_msg::geometry_msgs::PolygonStamped| {
                let mut unlocked_data = cloned_data.write().unwrap();
                unlocked_data.polygon_stamped_msg = Some(msg);
                unlocked_data.update();
            },
        )
        .unwrap();

        return PolygonListener {
            _data: data,
            _subscriber: sub,
        };
    }

    pub fn get_lines(&self) -> Vec<Line> {
        return self._data.clone().read().unwrap().get_lines();
    }
}
