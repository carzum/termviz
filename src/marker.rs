//! Module dealing with the reception, projection and lifecycle of visualization markers.
//!
//! ROS has a type of message dedicated to visualization: visualization_msgs::Marker.
//! This module allows to subsribe to topics that publish them and project them into the
//! 2D plane. Finally, it takes care of their lifecycle: ADD, DELETE and timeout.
use crate::config::ListenerConfig;
use nalgebra::geometry::Isometry3;
use std::collections::BTreeMap;
use std::f64::consts::PI;
use std::sync::{Arc, Mutex, RwLock};

use rosrust;
use rustros_tf::transforms::nalgebra::geometry::Point3;
use rustros_tf::transforms::{isometry_from_pose, isometry_from_transform};

use tui::style::Color;
use tui::widgets::canvas::Line;

struct TermvizMarker {
    pub lines: Vec<Line>,
    pub id: i32,
}

///Creates a list of lines from N line stips.
/// Arguments:
/// - `strips`: A vector of vector of points. Each element is a strip, i.e. a single
///             broken line. Each strip has N points that form N-1 lines.
fn from_point_strips(strips: &Vec<Vec<Point3<f64>>>, color: &Color) -> Vec<Line> {
    let mut lines: Vec<Line> = Vec::new();

    for strip in strips {
        let mut previous_point: Option<&Point3<f64>> = None;
        for point in strip {
            if previous_point.is_some() {
                let pp = previous_point.unwrap();
                lines.push(Line {
                    x1: pp.x,
                    y1: pp.y,
                    x2: point.x,
                    y2: point.y,
                    color: *color,
                });
            }
            previous_point = Some(point);
        }
    }

    lines
}

///Creates the visible lines for a cube.
///
/// If the cube is parallel to the floor, the visible lines are simply the 4 top ones.
/// Else, we draw all 12 edges.
/// Arguments:
/// - `dimension`: size of the cube as width, length, height.
/// - `offset`: Offset of the center of the cube in the iso transformation.
/// - `color`: Color of the cube.
/// - `iso`: Base transformation of the cube.
fn parse_cube(
    dimension: &rosrust_msg::geometry_msgs::Vector3,
    offset: &rosrust_msg::geometry_msgs::Point,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    let angles = iso.rotation.euler_angles();
    let width = dimension.x / 2.0;
    let length = dimension.y / 2.0;
    let height = dimension.z / 2.0;

    let pw = offset.x + width;
    let mw = offset.x - width;
    let pl = offset.y + length;
    let ml = offset.y - length;

    let mut pointss: Vec<Vec<Point3<f64>>> = Vec::new();

    if angles.0.abs() < 0.0001 && angles.1.abs() < 0.0001 {
        // cube about parallel to floor, only draw the top
        let points = vec![
            iso.transform_point(&Point3::new(pw, pl, 0.0)),
            iso.transform_point(&Point3::new(pw, ml, 0.0)),
            iso.transform_point(&Point3::new(mw, ml, 0.0)),
            iso.transform_point(&Point3::new(mw, pl, 0.0)),
            iso.transform_point(&Point3::new(pw, pl, 0.0)),
        ];
        pointss.push(points)
    } else {
        // Other faces may be visible, render all of them
        let ph = offset.z + height;
        let mh = offset.z - height;
        let face_top = vec![
            iso.transform_point(&Point3::new(pw, pl, ph)),
            iso.transform_point(&Point3::new(pw, ml, ph)),
            iso.transform_point(&Point3::new(mw, ml, ph)),
            iso.transform_point(&Point3::new(mw, pl, ph)),
            iso.transform_point(&Point3::new(mw, pl, ph)),
        ];
        let face_bottom = vec![
            iso.transform_point(&Point3::new(pw, pl, mh)),
            iso.transform_point(&Point3::new(pw, ml, mh)),
            iso.transform_point(&Point3::new(mw, ml, mh)),
            iso.transform_point(&Point3::new(mw, pl, mh)),
            iso.transform_point(&Point3::new(mw, pl, mh)),
        ];
        let a = vec![
            iso.transform_point(&Point3::new(pw, pl, ph)),
            iso.transform_point(&Point3::new(pw, pl, mh)),
        ];
        let b = vec![
            iso.transform_point(&Point3::new(mw, pl, ph)),
            iso.transform_point(&Point3::new(mw, pl, mh)),
        ];
        let c = vec![
            iso.transform_point(&Point3::new(pw, ml, ph)),
            iso.transform_point(&Point3::new(pw, ml, mh)),
        ];
        let d = vec![
            iso.transform_point(&Point3::new(mw, ml, ph)),
            iso.transform_point(&Point3::new(mw, ml, mh)),
        ];
        pointss.extend([face_top, face_bottom, a, b, c, d]);
    }

    return from_point_strips(&pointss, color);
}

fn parse_arrow_msg(
    msg: &rosrust_msg::visualization_msgs::Marker,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    let lines = match msg.points.len() {
        0 => {
            let mut lines: Vec<Line> = Vec::new();
            // method 1: Position/Orientation -> scale is the arrow dimension
            let p1 = iso.transform_point(&Point3::new(0.0, 0.0, 0.0));
            let p2 = iso.transform_point(&Point3::new(msg.scale.x, 0.0, 0.0));
            lines.push(Line {
                x1: p1.x,
                y1: p1.y,
                x2: p2.x,
                y2: p2.y,
                color: *color,
            });
            let angle = PI / 4.0;
            let r = msg.scale.y / 2.0 / angle.cos();
            let a = PI - angle;
            lines.push(Line {
                x1: p2.x,
                y1: p2.y,
                x2: p2.x + r * a.cos(),
                y2: p2.y + r * a.sin(),
                color: *color,
            });
            lines.push(Line {
                x1: p2.x,
                y1: p2.y,
                x2: p2.x - r * a.cos(),
                y2: p2.y - r * a.sin(),
                color: *color,
            });
            lines
        }
        2 => {
            // method 2: Start/End Pose -> 2 points controling the size
            // scale x is arrow width (not used), y head width, z head length
            let start = &msg.points[0];
            let end = &msg.points[1];

            let p1 = iso.transform_point(&Point3::new(start.x, start.y, start.z));
            let p2 = iso.transform_point(&Point3::new(end.x, end.y, end.z));

            let mut lines: Vec<Line> = Vec::new();
            lines.push(Line {
                x1: p1.x,
                y1: p1.y,
                x2: p2.x,
                y2: p2.y,
                color: *color,
            });

            let shaft_angle = (p2.y - p1.y).atan2(p2.x - p1.x);
            let angle = msg.scale.y.atan2(2.0 * msg.scale.x);
            let r = (msg.scale.x.powi(2) + msg.scale.y.powi(2)).sqrt();
            let a = shaft_angle + PI - angle;
            let b = shaft_angle + PI + angle;
            lines.push(Line {
                x1: p2.x,
                y1: p2.y,
                x2: p2.x + r * a.cos(),
                y2: p2.y + r * a.sin(),
                color: *color,
            });
            lines.push(Line {
                x1: p2.x,
                y1: p2.y,
                x2: p2.x + r * b.cos(),
                y2: p2.y + r * b.sin(),
                color: *color,
            });

            lines
        }
        _ => Vec::new(),
    };

    lines
}

fn parse_cube_msg(
    msg: &rosrust_msg::visualization_msgs::Marker,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    return parse_cube(
        &msg.scale,
        &rosrust_msg::geometry_msgs::Point {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        },
        color,
        iso,
    );
}

fn parse_cube_list_msg(
    msg: &rosrust_msg::visualization_msgs::Marker,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    let mut lines = Vec::new();

    for point in msg.points.iter() {
        lines.extend(parse_cube(&msg.scale, &point, color, iso));
    }

    lines
}

fn parse_line_strip_msg(
    msg: &rosrust_msg::visualization_msgs::Marker,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    let mut points: Vec<Point3<f64>> = Vec::new();

    for point in msg.points.iter() {
        points.push(iso.transform_point(&Point3::new(point.x, point.y, point.z)));
    }

    return from_point_strips(&vec![points], color);
}

fn parse_line_list_msg(
    msg: &rosrust_msg::visualization_msgs::Marker,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    let mut lines: Vec<Line> = Vec::new();

    let mut point_it = msg.points.iter();

    while let Some(msg_p1) = point_it.next() {
        let p1 = iso.transform_point(&Point3::new(msg_p1.x, msg_p1.y, msg_p1.z));
        let msg_p2 = point_it.next().expect("Malformed message.");
        let p2 = iso.transform_point(&Point3::new(msg_p2.x, msg_p2.y, msg_p2.z));

        lines.push(Line {
            x1: p1.x,
            y1: p1.y,
            x2: p2.x,
            y2: p2.y,
            color: *color,
        });
    }
    lines
}

fn parse_marker_msg(
    msg: &rosrust_msg::visualization_msgs::Marker,
    tf: &rosrust_msg::geometry_msgs::Transform,
) -> TermvizMarker {
    let trans_marker_to_static_frame = isometry_from_transform(tf);
    let trans_to_marker = isometry_from_pose(&msg.pose);

    let iso = trans_marker_to_static_frame.inverse() * trans_to_marker;

    let color = Color::Rgb(
        (msg.color.r * 255.0) as u8,
        (msg.color.g * 255.0) as u8,
        (msg.color.b * 255.0) as u8,
    );

    let res = match msg.type_ as u8 {
        rosrust_msg::visualization_msgs::Marker::ARROW => parse_arrow_msg(msg, &color, &iso),
        rosrust_msg::visualization_msgs::Marker::CUBE => parse_cube_msg(msg, &color, &iso),
        rosrust_msg::visualization_msgs::Marker::CUBE_LIST => {
            parse_cube_list_msg(msg, &color, &iso)
        }
        rosrust_msg::visualization_msgs::Marker::LINE_STRIP => {
            parse_line_strip_msg(msg, &color, &iso)
        }
        rosrust_msg::visualization_msgs::Marker::LINE_LIST => {
            parse_line_list_msg(msg, &color, &iso)
        }
        _ => Vec::new(),
    };

    TermvizMarker {
        lines: res,
        id: msg.id,
    }
}

///Class that holds all the markers currently active.
///
/// The markers are ordered in a double dictionary, which allows to manage namespaces.
/// Each namespace has its own set of IDs. This structure allows to have markers with
/// same IDs on different namespaces. Note that this class should be shared by all
/// publishers such that they can be managed globally.
struct TermvizMarkerContainer {
    _markers: BTreeMap<String, BTreeMap<i32, TermvizMarker>>,
    _static_frame: String,
    _tf_listener: Arc<rustros_tf::TfListener>,
}

impl TermvizMarkerContainer {
    pub fn new(
        tf_listener: Arc<rustros_tf::TfListener>,
        static_frame: String,
    ) -> TermvizMarkerContainer {
        Self {
            _markers: BTreeMap::<String, BTreeMap<i32, TermvizMarker>>::new(),
            _static_frame: static_frame,
            _tf_listener: tf_listener,
        }
    }

    fn add_marker(&mut self, marker: &rosrust_msg::visualization_msgs::Marker) {
        let transform = &self._tf_listener.clone().lookup_transform(
            &marker.header.frame_id,
            &self._static_frame.clone(),
            marker.header.stamp,
        );
        match &transform {
            Ok(transform) => transform,
            Err(_e) => return,
        };

        self._markers
            .entry(marker.ns.clone())
            .and_modify(|namespace| {
                let res = parse_marker_msg(&marker, &transform.as_ref().unwrap().transform);
                namespace.insert(res.id, res);
            })
            .or_insert_with(|| {
                let res = parse_marker_msg(&marker, &transform.as_ref().unwrap().transform);
                let mut namespace = BTreeMap::<i32, TermvizMarker>::new();
                namespace.insert(res.id, res);
                namespace
            });
    }

    fn delete_marker(&mut self, marker_ns: String, marker_id: i32) {
        self._markers.entry(marker_ns).and_modify(|namespace| {
            namespace.remove(&marker_id);
        });
    }

    fn clear(&mut self) {
        self._markers.clear();
    }

    fn clear_namespace(&mut self, marker_ns: String) -> Vec<i32> {
        let mut res = Vec::new();
        self._markers.entry(marker_ns).and_modify(|namespace| {
            for marker in namespace.values() {
                res.push(marker.id);
            }
            namespace.clear();
        });
        res
    }

    fn get_lines(&self) -> Vec<Line> {
        let mut res = Vec::<Line>::new();
        for namespace in self._markers.values() {
            for marker in namespace.values() {
                res.extend(marker.lines.to_vec());
            }
        }
        res
    }
}

///Class that handles the lifecycle of the markers.
///
/// Markers that provide a lifecycle (i.e. not 0) need to be deleted from the container
/// once the lifecycle is reached. To do this we add another container which has a timer.
/// The timer collects jobs after a delay to delete markers from the markers container.
///
/// Because the timer uses guards, we also need to store them which is done with a dict
/// and a list. A repeated task is given to the timer to consume the list of deleted
/// marker and free the guards.
struct MarkersLifecycle {
    _markers_container: Arc<RwLock<TermvizMarkerContainer>>,
    _deleted_markers: Arc<Mutex<Vec<(String, i32)>>>,
    _guards: Arc<Mutex<BTreeMap<(String, i32), timer::Guard>>>,
    _timer: Arc<Mutex<timer::Timer>>,
    _cleaner_guard: timer::Guard,
}

impl MarkersLifecycle {
    pub fn new(marker_container: TermvizMarkerContainer) -> MarkersLifecycle {
        let timer = Arc::new(Mutex::new(timer::Timer::new()));
        let deleted_markers = Arc::new(Mutex::new(Vec::<(String, i32)>::new()));
        let guards = Arc::new(Mutex::new(BTreeMap::<(String, i32), timer::Guard>::new()));

        let clean_job_delete_markers_ref = deleted_markers.clone();
        let clean_job_guards_ref = guards.clone();

        let cleaner_guard =
            timer
                .lock()
                .unwrap()
                .schedule_repeating(chrono::Duration::seconds(5), move || {
                    // every 5s clear the guards that outlived their markers
                    let mut clean_job_delete_markers = clean_job_delete_markers_ref.lock().unwrap();
                    let mut clean_job_guards = clean_job_guards_ref.lock().unwrap();
                    for key in clean_job_delete_markers.iter() {
                        clean_job_guards.remove(key);
                    }
                    clean_job_delete_markers.clear();
                });

        Self {
            _markers_container: Arc::new(RwLock::new(marker_container)),
            _deleted_markers: deleted_markers,
            _guards: guards,
            _timer: timer,
            _cleaner_guard: cleaner_guard,
        }
    }

    fn add_marker(&mut self, marker: &rosrust_msg::visualization_msgs::Marker) {
        self._markers_container.write().unwrap().add_marker(marker);

        // Handle marker lifecycle
        if marker.lifetime.seconds() == 0.0 {
            return;
        }

        let markers_container_ref = self._markers_container.clone();

        let chrono_delay = chrono::Duration::seconds(marker.lifetime.sec as i64)
            + chrono::Duration::nanoseconds(marker.lifetime.nsec as i64);

        let marker_info = (marker.ns.clone(), marker.id);

        let guard_ = self
            ._timer
            .lock()
            .unwrap()
            .schedule_with_delay(chrono_delay, move || {
                let mut mc = markers_container_ref.write().unwrap();
                mc.delete_marker(marker_info.0.clone(), marker_info.1);
            });

        let mut guards_ref = self._guards.lock().unwrap();
        guards_ref.insert((marker.ns.clone(), marker.id), guard_);
    }

    fn delete_marker(&mut self, marker_ns: String, marker_id: i32) {
        self._markers_container
            .write()
            .unwrap()
            .delete_marker(marker_ns.clone(), marker_id);
        self._deleted_markers
            .lock()
            .unwrap()
            .push((marker_ns, marker_id));
    }

    fn clear(&mut self) {
        // loose the guards if any, to cancel the timer tasks
        self._deleted_markers.lock().unwrap().clear();
        self._guards.lock().unwrap().clear();

        self._markers_container.write().unwrap().clear();
    }

    fn clear_namespace(&mut self, marker_ns: String) {
        let removed_ids = self
            ._markers_container
            .write()
            .unwrap()
            .clear_namespace(marker_ns.clone());

        // schedule deletion of the cleared markers
        let mut deleted_markers = self._deleted_markers.lock().unwrap();
        for id in removed_ids {
            deleted_markers.push((marker_ns.clone(), id));
        }
    }

    fn get_lines(&self) -> Vec<Line> {
        self._markers_container.write().unwrap().get_lines()
    }
}

pub struct MarkersListener {
    _markers_lifecycle: Arc<RwLock<MarkersLifecycle>>,
    _subscribers: Vec<Arc<Mutex<rosrust::Subscriber>>>,
}

impl MarkersListener {
    pub fn new(tf_listener: Arc<rustros_tf::TfListener>, static_frame: String) -> MarkersListener {
        let marker_container = TermvizMarkerContainer::new(tf_listener, static_frame);
        Self {
            _markers_lifecycle: Arc::new(RwLock::new(MarkersLifecycle::new(marker_container))),
            _subscribers: Vec::new(),
        }
    }

    /// Gets all the lines currently active, to render.
    pub fn get_lines(&self) -> Vec<Line> {
        let markers_container_ref = self._markers_lifecycle.read().unwrap();
        markers_container_ref.get_lines()
    }

    /// Adds a subscriber for a marker topic.
    ///
    /// Arguments:
    /// - `config`: Configuration containing the topic name.
    pub fn add_marker_listener(&mut self, config: &ListenerConfig) {
        let markers_container_ref = self._markers_lifecycle.clone();

        let sub = rosrust::subscribe(
            &config.topic,
            2,
            move |msg: rosrust_msg::visualization_msgs::Marker| {
                let mut markers_container = markers_container_ref.write().unwrap();

                match msg.action as u8 {
                    rosrust_msg::visualization_msgs::Marker::ADD => {
                        markers_container.add_marker(&msg)
                    }
                    rosrust_msg::visualization_msgs::Marker::DELETE => {
                        markers_container.delete_marker(msg.ns.clone(), msg.id)
                    }
                    rosrust_msg::visualization_msgs::Marker::DELETEALL => markers_container.clear(),
                    _ => return,
                }
            },
        );

        self._subscribers.push(Arc::new(Mutex::new(sub.unwrap())));
    }

    /// Adds a subscriber for a marker array message topic.
    ///
    /// # Arguments
    /// * `config` - Configuration containing the topic.
    pub fn add_marker_array_listener(&mut self, config: &ListenerConfig) {
        let markers_container_ref = self._markers_lifecycle.clone();

        let sub = rosrust::subscribe(
            &config.topic,
            2,
            move |msg: rosrust_msg::visualization_msgs::MarkerArray| {
                let mut markers_container = markers_container_ref.write().unwrap();

                for marker in msg.markers {
                    match marker.action as u8 {
                        rosrust_msg::visualization_msgs::Marker::ADD => {
                            markers_container.add_marker(&marker)
                        }
                        rosrust_msg::visualization_msgs::Marker::DELETE => {
                            markers_container.delete_marker(marker.ns.clone(), marker.id)
                        }
                        rosrust_msg::visualization_msgs::Marker::DELETEALL => {
                            markers_container.clear_namespace(marker.ns.clone())
                        }
                        _ => continue,
                    }
                }
            },
        );

        self._subscribers.push(Arc::new(Mutex::new(sub.unwrap())));
    }
}
