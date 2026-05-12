//! Module dealing with the reception, projection and lifecycle of visualization markers.
//!
//! ROS has a type of message dedicated to visualization: visualization_msgs::Marker.
//! This module allows to subsribe to topics that publish them and project them into the
//! 2D plane. Finally, it takes care of their lifecycle: ADD, DELETE and timeout.
use crate::config::ListenerConfig;
use crate::ros;
use crate::ros::tf::TfClient;
use crate::ros::types;
use crate::transformation::{ros_pose_to_isometry, ros_transform_to_isometry};
use nalgebra::base::Vector3;
use nalgebra::geometry::Isometry3;
use std::collections::HashMap;
use std::f64::consts::PI;
use std::sync::{Arc, Mutex, RwLock};

use nalgebra::geometry::Point3;

use tui::style::Color;
use tui::widgets::canvas::Line;

struct TermvizMarker {
    pub lines: Vec<Line>,
    pub id: i32,
}

/// Creates a list of lines from N line strips.
/// # Arguments
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

/// Creates the visible lines for a cube.
///
/// If the cube is parallel to the plan, the visible lines are simoffset.y + dimension.y / 2.0y the 4 top ones.
/// Else, we draw all 12 edges.
/// # Arguments:
/// - `dimension`: size of the cube as width, length, height.
/// - `offset`: Offset of the center of the cube in the iso transformation.
/// - `color`: Color of the cube.
/// - `iso`: Base transformation of the cube.
fn parse_cube(
    dimension: &types::Vector3,
    offset: &types::Point,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    let angles = iso.rotation.euler_angles();

    let mut points_strips: Vec<Vec<Point3<f64>>> = Vec::new();

    let face_top = vec![
        iso.transform_point(&Point3::new(
            offset.x + dimension.x / 2.0,
            offset.y + dimension.y / 2.0,
            offset.z + dimension.z / 2.0,
        )),
        iso.transform_point(&Point3::new(
            offset.x + dimension.x / 2.0,
            offset.y - dimension.y / 2.0,
            offset.z + dimension.z / 2.0,
        )),
        iso.transform_point(&Point3::new(
            offset.x - dimension.x / 2.0,
            offset.y - dimension.y / 2.0,
            offset.z + dimension.z / 2.0,
        )),
        iso.transform_point(&Point3::new(
            offset.x - dimension.x / 2.0,
            offset.y + dimension.y / 2.0,
            offset.z + dimension.z / 2.0,
        )),
        iso.transform_point(&Point3::new(
            offset.x + dimension.x / 2.0,
            offset.y + dimension.y / 2.0,
            offset.z + dimension.z / 2.0,
        )),
    ];
    points_strips.push(face_top);

    if angles.0.abs() > 0.0001 || angles.1.abs() > 0.0001 {
        // Other faces may be visible, render all of them
        let face_bottom = vec![
            iso.transform_point(&Point3::new(
                offset.x + dimension.x / 2.0,
                offset.y + dimension.y / 2.0,
                offset.z - dimension.z / 2.0,
            )),
            iso.transform_point(&Point3::new(
                offset.x + dimension.x / 2.0,
                offset.y - dimension.y / 2.0,
                offset.z - dimension.z / 2.0,
            )),
            iso.transform_point(&Point3::new(
                offset.x - dimension.x / 2.0,
                offset.y - dimension.y / 2.0,
                offset.z - dimension.z / 2.0,
            )),
            iso.transform_point(&Point3::new(
                offset.x - dimension.x / 2.0,
                offset.y + dimension.y / 2.0,
                offset.z - dimension.z / 2.0,
            )),
            iso.transform_point(&Point3::new(
                offset.x + dimension.x / 2.0,
                offset.y + dimension.y / 2.0,
                offset.z - dimension.z / 2.0,
            )),
        ];
        let a = vec![
            iso.transform_point(&Point3::new(
                offset.x + dimension.x / 2.0,
                offset.y + dimension.y / 2.0,
                offset.z + dimension.z / 2.0,
            )),
            iso.transform_point(&Point3::new(
                offset.x + dimension.x / 2.0,
                offset.y + dimension.y / 2.0,
                offset.z - dimension.z / 2.0,
            )),
        ];
        let b = vec![
            iso.transform_point(&Point3::new(
                offset.x - dimension.x / 2.0,
                offset.y + dimension.y / 2.0,
                offset.z + dimension.z / 2.0,
            )),
            iso.transform_point(&Point3::new(
                offset.x - dimension.x / 2.0,
                offset.y + dimension.y / 2.0,
                offset.z - dimension.z / 2.0,
            )),
        ];
        let c = vec![
            iso.transform_point(&Point3::new(
                offset.x + dimension.x / 2.0,
                offset.y - dimension.y / 2.0,
                offset.z + dimension.z / 2.0,
            )),
            iso.transform_point(&Point3::new(
                offset.x + dimension.x / 2.0,
                offset.y - dimension.y / 2.0,
                offset.z - dimension.z / 2.0,
            )),
        ];
        let d = vec![
            iso.transform_point(&Point3::new(
                offset.x - dimension.x / 2.0,
                offset.y - dimension.y / 2.0,
                offset.z + dimension.z / 2.0,
            )),
            iso.transform_point(&Point3::new(
                offset.x - dimension.x / 2.0,
                offset.y - dimension.y / 2.0,
                offset.z - dimension.z / 2.0,
            )),
        ];
        points_strips.extend([face_bottom, a, b, c, d]);
    }

    return from_point_strips(&points_strips, color);
}

fn parse_arrow_msg(
    msg: &types::Marker,
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
            //calculate 2 points representing the end of the head
            let angle = PI / 4.0;
            let r = msg.scale.y / 2.0 / angle.cos();
            let a = PI - angle;
            let b = PI + angle;
            // transform points
            let p3_right =
                iso.transform_point(&Point3::new(msg.scale.x + r * a.cos(), r * a.sin(), 0.0));
            let p3_left =
                iso.transform_point(&Point3::new(msg.scale.x + r * b.cos(), r * b.sin(), 0.0));
            lines.push(Line {
                x1: p2.x,
                y1: p2.y,
                x2: p3_right.x,
                y2: p3_right.y,
                color: *color,
            });
            lines.push(Line {
                x1: p2.x,
                y1: p2.y,
                x2: p3_left.x,
                y2: p3_left.y,
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

            // get angle at the head of the arrow and calculate head's lines in head transform
            let head_trafo = Isometry3::face_towards(&p2, &p1, &Vector3::y());
            let half_head_width = msg.scale.y / 2.0;
            let angle = half_head_width.atan2(msg.scale.z);
            let r = (half_head_width.powi(2) + msg.scale.z.powi(2)).sqrt();

            let p3_right =
                head_trafo.transform_point(&Point3::new(0.0, r * angle.sin(), r * angle.cos()));
            let p3_left =
                head_trafo.transform_point(&Point3::new(0.0, -r * angle.sin(), r * angle.cos()));
            lines.push(Line {
                x1: p2.x,
                y1: p2.y,
                x2: p3_right.x,
                y2: p3_right.y,
                color: *color,
            });
            lines.push(Line {
                x1: p2.x,
                y1: p2.y,
                x2: p3_left.x,
                y2: p3_left.y,
                color: *color,
            });

            lines
        }
        _ => Vec::new(),
    };

    lines
}

fn parse_cube_msg(
    msg: &types::Marker,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    let center_offset_msg = msg.points.get(0);
    if center_offset_msg.is_none() {
        return parse_cube(
            &msg.scale,
            &types::Point { x: 0.0, y: 0.0, z: 0.0 },
            color,
            iso,
        );
    }
    return parse_cube(&msg.scale, &center_offset_msg.unwrap(), color, iso);
}

fn parse_cube_list_msg(
    msg: &types::Marker,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    let mut lines = Vec::new();

    for point in msg.points.iter() {
        lines.extend(parse_cube(&msg.scale, &point, color, iso));
    }

    lines
}

fn parse_points_msg(
    msg: &types::Marker,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    return parse_cube_list_msg(msg, color, iso);
}

fn parse_line_strip_msg(
    msg: &types::Marker,
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
    msg: &types::Marker,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    let mut lines: Vec<Line> = Vec::new();

    let mut point_it = msg.points.iter();
    let mut color_it = msg.colors.iter();

    while let Some(msg_p1) = point_it.next() {
        let msg_color = color_it.next();
        let local_color = match msg_color {
            Some(x) => Color::Rgb(
                (x.r * 255.0) as u8,
                (x.g * 255.0) as u8,
                (x.b * 255.0) as u8,
            ),
            None => *color,
        };
        color_it.next(); // these come in pairs, but I currently don't see the necessity to implement gradients

        let p1 = iso.transform_point(&Point3::new(msg_p1.x, msg_p1.y, msg_p1.z));
        let msg_p2 = point_it.next().expect("Malformed message.");
        let p2 = iso.transform_point(&Point3::new(msg_p2.x, msg_p2.y, msg_p2.z));

        lines.push(Line {
            x1: p1.x,
            y1: p1.y,
            x2: p2.x,
            y2: p2.y,
            color: local_color,
        });
    }
    lines
}

fn parse_sphere_msg(
    msg: &types::Marker,
    color: &tui::style::Color,
    iso: &Isometry3<f64>,
) -> Vec<Line> {
    let mut lines: Vec<Line> = Vec::new();

    let scale = &Point3::new(msg.scale.x, msg.scale.y, msg.scale.z);

    let p1 = iso.transform_point(&Point3::new(-scale.x * 0.5, 0.0, 0.0));
    let p2 = iso.transform_point(&Point3::new(scale.x * 0.5, 0.0, 0.0));
    let p3 = iso.transform_point(&Point3::new(0.0, -scale.y * 0.5, 0.0));
    let p4 = iso.transform_point(&Point3::new(0.0, scale.y * 0.5, 0.0));
    let p5 = iso.transform_point(&Point3::new(0.0, 0.0, -scale.z * 0.5));
    let p6 = iso.transform_point(&Point3::new(0.0, 0.0, scale.z * 0.5));

    //central "cross" showing the main axes and the center
    lines.push(Line {
        x1: p1.x,
        y1: p1.y,
        x2: p2.x,
        y2: p2.y,
        color: *color,
    });
    lines.push(Line {
        x1: p3.x,
        y1: p3.y,
        x2: p4.x,
        y2: p4.y,
        color: *color,
    });
    lines.push(Line {
        x1: p5.x,
        y1: p5.y,
        x2: p6.x,
        y2: p6.y,
        color: *color,
    });

    let segment_count = 20; //number of segments of each ellipse for a pair of axes
    let step = (2.0 * PI) / (segment_count as f64);
    for i in 0..segment_count {
        //ellipse around XY cut
        let ifl = i as f64; //iteration number as float
        let pa = iso.transform_point(&Point3::new(
            0.5 * scale.x * (ifl * (step)).sin(),
            0.5 * scale.y * (ifl * (step)).cos(),
            0.0,
        ));
        let pb = iso.transform_point(&Point3::new(
            0.5 * scale.x * ((ifl + 1.0) * (step)).sin(),
            0.5 * scale.y * ((ifl + 1.0) * (step)).cos(),
            0.0,
        ));
        lines.push(Line {
            x1: pa.x,
            y1: pa.y,
            x2: pb.x,
            y2: pb.y,
            color: *color,
        });
    }
    for i in 0..segment_count {
        //ellipse around XZ cut
        let ifl = i as f64; //iteration number as float
        let pa = iso.transform_point(&Point3::new(
            0.5 * scale.x * (ifl * (step)).sin(),
            0.0,
            0.5 * scale.z * (ifl * (step)).cos(),
        ));
        let pb = iso.transform_point(&Point3::new(
            0.5 * scale.x * ((ifl + 1.0) * (step)).sin(),
            0.0,
            0.5 * scale.z * ((ifl + 1.0) * (step)).cos(),
        ));
        lines.push(Line {
            x1: pa.x,
            y1: pa.y,
            x2: pb.x,
            y2: pb.y,
            color: *color,
        });
    }
    for i in 0..segment_count {
        //ellipse around YZ cut
        let ifl = i as f64; //iteration number as float
        let pa = iso.transform_point(&Point3::new(
            0.0,
            0.5 * scale.y * ((ifl * (step)).cos()),
            0.5 * scale.z * ((ifl * (step)).sin()),
        ));
        let pb = iso.transform_point(&Point3::new(
            0.0,
            0.5 * scale.y * (((ifl + 1.0) * (step)).cos()),
            0.5 * scale.z * (((ifl + 1.0) * (step)).sin()),
        ));
        lines.push(Line {
            x1: pa.x,
            y1: pa.y,
            x2: pb.x,
            y2: pb.y,
            color: *color,
        });
    }

    lines
}

fn parse_marker_msg(
    msg: &types::Marker,
    tf: &types::Transform,
) -> TermvizMarker {
    let trans_marker_to_static_frame = ros_transform_to_isometry(tf);
    let trans_to_marker = ros_pose_to_isometry(&msg.pose);

    let iso = trans_marker_to_static_frame.inverse() * trans_to_marker;

    let color = Color::Rgb(
        (msg.color.r * 255.0) as u8,
        (msg.color.g * 255.0) as u8,
        (msg.color.b * 255.0) as u8,
    );

    let res = match msg.type_ {
        types::Marker::ARROW => parse_arrow_msg(msg, &color, &iso),
        types::Marker::CUBE => parse_cube_msg(msg, &color, &iso),
        types::Marker::CUBE_LIST => parse_cube_list_msg(msg, &color, &iso),
        types::Marker::POINTS => parse_points_msg(msg, &color, &iso),
        types::Marker::LINE_STRIP => parse_line_strip_msg(msg, &color, &iso),
        types::Marker::LINE_LIST => parse_line_list_msg(msg, &color, &iso),
        types::Marker::SPHERE => parse_sphere_msg(msg, &color, &iso),
        _ => Vec::new(),
    };

    TermvizMarker {
        lines: res,
        id: msg.id,
    }
}

/// Class that holds all the markers currently active.
///
/// The markers are ordered in a double dictionary, which allows to manage namespaces.
/// Each namespace has its own set of IDs. This structure allows to have markers with
/// same IDs on different namespaces. Note that this class should be shared by all
/// publishers such that they can be managed globally.
struct TermvizMarkerContainer {
    markers: HashMap<String, HashMap<i32, TermvizMarker>>,
    static_frame: String,
    tf: Arc<dyn TfClient>,
}

impl TermvizMarkerContainer {
    pub fn new(
        tf: Arc<dyn TfClient>,
        static_frame: String,
    ) -> TermvizMarkerContainer {
        Self {
            markers: HashMap::<String, HashMap<i32, TermvizMarker>>::new(),
            static_frame: static_frame,
            tf,
        }
    }

    fn add_marker(&mut self, marker: &types::Marker) {
        let transform = &self.tf.lookup_transform(
            &marker.header.frame_id,
            &self.static_frame.clone(),
            marker.header.stamp,
        );
        match &transform {
            Ok(transform) => transform,
            Err(_e) => return,
        };

        self.markers
            .entry(marker.ns.clone())
            .and_modify(|namespace| {
                let res = parse_marker_msg(&marker, &transform.as_ref().unwrap().transform);
                namespace.insert(res.id, res);
            })
            .or_insert_with(|| {
                let res = parse_marker_msg(&marker, &transform.as_ref().unwrap().transform);
                let mut namespace = HashMap::<i32, TermvizMarker>::new();
                namespace.insert(res.id, res);
                namespace
            });
    }

    fn delete_marker(&mut self, marker_ns: String, marker_id: i32) {
        self.markers.entry(marker_ns).and_modify(|namespace| {
            namespace.remove(&marker_id);
        });
    }

    fn clear(&mut self) {
        self.markers.clear();
    }

    fn clear_namespace(&mut self, marker_ns: String) -> Vec<i32> {
        let mut res = Vec::new();
        self.markers.entry(marker_ns).and_modify(|namespace| {
            for marker in namespace.values() {
                res.push(marker.id);
            }
            namespace.clear();
        });
        res
    }

    fn get_lines(&self) -> Vec<Line> {
        let mut res = Vec::<Line>::new();
        for namespace in self.markers.values() {
            for marker in namespace.values() {
                res.extend(marker.lines.to_vec());
            }
        }
        res
    }
}

/// Class that handles the lifecycle of the markers.
///
/// Markers that provide a lifecycle (i.e. not 0) need to be deleted from the container
/// once the lifecycle is reached. To do this we add another container which has a timer.
/// The timer collects jobs after a delay to delete markers from the markers container.
///
/// Because the timer uses guards, we also need to store them which is done with a dict
/// and a list. A repeated task is given to the timer to consume the list of deleted
/// marker and free the guards.
struct MarkersLifecycle {
    markers_container: Arc<RwLock<TermvizMarkerContainer>>,
    deleted_markers: Arc<Mutex<Vec<(String, i32)>>>,
    guards: Arc<Mutex<HashMap<(String, i32), timer::Guard>>>,
    timer: Arc<Mutex<timer::Timer>>,
    #[allow(dead_code)] // because the guard is never used but must be kept
    cleaner_guard: timer::Guard,
}

impl MarkersLifecycle {
    pub fn new(marker_container: TermvizMarkerContainer) -> MarkersLifecycle {
        let timer = Arc::new(Mutex::new(timer::Timer::new()));
        let deleted_markers = Arc::new(Mutex::new(Vec::<(String, i32)>::new()));
        let guards = Arc::new(Mutex::new(HashMap::<(String, i32), timer::Guard>::new()));

        let clean_job_delete_markers_ref = deleted_markers.clone();
        let clean_job_guards_ref = guards.clone();

        let guard =
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
            markers_container: Arc::new(RwLock::new(marker_container)),
            deleted_markers: deleted_markers,
            guards: guards,
            timer: timer,
            cleaner_guard: guard,
        }
    }

    fn add_marker(&mut self, marker: &types::Marker) {
        self.markers_container.write().unwrap().add_marker(marker);

        // Handle marker lifecycle
        if marker.lifetime.sec == 0 && marker.lifetime.nsec == 0 {
            return;
        }

        let markers_container_ref = self.markers_container.clone();

        let chrono_delay = chrono::Duration::seconds(marker.lifetime.sec as i64)
            + chrono::Duration::nanoseconds(marker.lifetime.nsec as i64);

        let marker_info = (marker.ns.clone(), marker.id);

        let guard_ = self
            .timer
            .lock()
            .unwrap()
            .schedule_with_delay(chrono_delay, move || {
                let mut mc = markers_container_ref.write().unwrap();
                mc.delete_marker(marker_info.0.clone(), marker_info.1);
            });

        let mut guards_ref = self.guards.lock().unwrap();
        guards_ref.insert((marker.ns.clone(), marker.id), guard_);
    }

    fn delete_marker(&mut self, marker_ns: String, marker_id: i32) {
        self.markers_container
            .write()
            .unwrap()
            .delete_marker(marker_ns.clone(), marker_id);
        self.deleted_markers
            .lock()
            .unwrap()
            .push((marker_ns, marker_id));
    }

    fn clear(&mut self) {
        // loose the guards if any, to cancel the timer tasks
        self.deleted_markers.lock().unwrap().clear();
        self.guards.lock().unwrap().clear();

        self.markers_container.write().unwrap().clear();
    }

    fn clear_namespace(&mut self, marker_ns: String) {
        let removed_ids = self
            .markers_container
            .write()
            .unwrap()
            .clear_namespace(marker_ns.clone());

        // schedule deletion of the cleared markers
        let mut deleted_markers = self.deleted_markers.lock().unwrap();
        deleted_markers.extend(removed_ids.iter().map(|&id| (marker_ns.clone(), id)));
    }

    fn get_lines(&self) -> Vec<Line> {
        self.markers_container.write().unwrap().get_lines()
    }
}

pub struct MarkersListener {
    markers_lifecycle: Arc<RwLock<MarkersLifecycle>>,
    subscribers: Vec<ros::SubscriptionHandle>,
}

impl MarkersListener {
    pub fn new(tf: Arc<dyn TfClient>, static_frame: String) -> MarkersListener {
        let marker_container = TermvizMarkerContainer::new(tf, static_frame);
        Self {
            markers_lifecycle: Arc::new(RwLock::new(MarkersLifecycle::new(marker_container))),
            subscribers: Vec::new(),
        }
    }

    /// Gets all the lines currently active, to render.
    pub fn get_lines(&self) -> Vec<Line> {
        let markers_container_ref = self.markers_lifecycle.read().unwrap();
        markers_container_ref.get_lines()
    }

    /// Adds a subscriber for a marker topic.
    ///
    /// # Arguments
    /// - `config`: Configuration containing the topic name.
    pub fn add_marker_listener(&mut self, config: &ListenerConfig) {
        let markers_container_ref = self.markers_lifecycle.clone();

        let sub = ros::subscribe_marker(&config.topic, 2, move |msg: types::Marker| {
                let mut markers_container = markers_container_ref.write().unwrap();

                match msg.action {
                    types::Marker::ADD => markers_container.add_marker(&msg),
                    types::Marker::DELETE => markers_container.delete_marker(msg.ns.clone(), msg.id),
                    types::Marker::DELETEALL => markers_container.clear(),
                    _ => return,
                }
            })
            .unwrap();

        self.subscribers.push(sub);
    }

    /// Adds a subscriber for a marker array message topic.
    ///
    /// # Arguments
    /// * `config` - Configuration containing the topic.
    pub fn add_marker_array_listener(&mut self, config: &ListenerConfig) {
        let markers_container_ref = self.markers_lifecycle.clone();

        let sub = ros::subscribe_marker_array(&config.topic, 2, move |msg: types::MarkerArray| {
                let mut markers_container = markers_container_ref.write().unwrap();

                for marker in msg.markers {
                    match marker.action {
                        types::Marker::ADD => markers_container.add_marker(&marker),
                        types::Marker::DELETE => {
                            markers_container.delete_marker(marker.ns.clone(), marker.id)
                        }
                        types::Marker::DELETEALL => markers_container.clear_namespace(marker.ns.clone()),
                        _ => continue,
                    }
                }
            })
            .unwrap();

        self.subscribers.push(sub);
    }
}
