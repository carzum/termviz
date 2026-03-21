use crate::ros::tf::{TfClient, TfError};
use crate::ros::types::*;
use futures::StreamExt;
use nalgebra::geometry::{Isometry3, Quaternion as NaQuaternion, Translation3, UnitQuaternion};
use serde::de::DeserializeOwned;
use std::collections::{HashMap, VecDeque};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, RwLock};
use std::thread;
use std::time::Duration as StdDuration;

pub struct Ros2Runtime {
    node: Arc<Mutex<r2r::Node>>,
    tf: Arc<TfBuffer>,
    spinner_running: Arc<AtomicBool>,
    spinner: Option<thread::JoinHandle<()>>,
}

impl Ros2Runtime {
    pub fn init(name: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let ctx = r2r::Context::create()?;
        let node = r2r::Node::create(ctx, name, "")?;
        let node = Arc::new(Mutex::new(node));

        let tf = Arc::new(TfBuffer::new());
        tf.clone().start(node.clone())?;

        let spinner_running = Arc::new(AtomicBool::new(true));
        let spinner = Some(start_spinner(node.clone(), spinner_running.clone()));

        Ok(Self {
            node,
            tf,
            spinner_running,
            spinner,
        })
    }

    pub fn tf_client(&self) -> Arc<dyn TfClient> {
        self.tf.clone()
    }

    pub fn now() -> Time {
        // r2r provides ROS clock access via node time APIs; keep it simple for now.
        Time::zero()
    }

    pub fn topics(&self) -> Vec<(String, String)> {
        let node = self.node.lock().unwrap();
        match node.get_topic_names_and_types() {
            Ok(map) => map
                .into_iter()
                .flat_map(|(name, types)| {
                    types
                        .into_iter()
                        .map(move |t| (name.clone(), t.clone()))
                })
                .collect(),
            Err(_) => Vec::new(),
        }
    }

    pub fn param_get<T: DeserializeOwned>(&self, _name: &str) -> Option<T> {
        // ROS2 param support is node-scoped; termviz previously queried a global ROS1 param.
        None
    }

    pub fn subscribe_laserscan(
        &self,
        topic: &str,
        _queue: usize,
        callback: impl Fn(LaserScan) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
        subscribe_laserscan(self.node.clone(), topic, callback)
    }

    pub fn subscribe_occupancy_grid(
        &self,
        topic: &str,
        _queue: usize,
        callback: impl Fn(OccupancyGrid) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
        subscribe_occupancy_grid(self.node.clone(), topic, callback)
    }

    pub fn subscribe_pointcloud2(
        &self,
        topic: &str,
        _queue: usize,
        callback: impl Fn(PointCloud2) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
        subscribe_pointcloud2(self.node.clone(), topic, callback)
    }

    pub fn subscribe_polygon_stamped(
        &self,
        topic: &str,
        _queue: usize,
        callback: impl Fn(PolygonStamped) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
        subscribe_polygon_stamped(self.node.clone(), topic, callback)
    }

    pub fn subscribe_image(
        &self,
        topic: &str,
        _queue: usize,
        callback: impl Fn(Image) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
        subscribe_image(self.node.clone(), topic, callback)
    }

    pub fn subscribe_marker(
        &self,
        topic: &str,
        _queue: usize,
        callback: impl Fn(Marker) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
        subscribe_marker(self.node.clone(), topic, callback)
    }

    pub fn subscribe_marker_array(
        &self,
        topic: &str,
        _queue: usize,
        callback: impl Fn(MarkerArray) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
        subscribe_marker_array(self.node.clone(), topic, callback)
    }

    pub fn subscribe_pose_stamped(
        &self,
        topic: &str,
        _queue: usize,
        callback: impl Fn(PoseStamped) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
        subscribe_pose_stamped(self.node.clone(), topic, callback)
    }

    pub fn subscribe_pose_array(
        &self,
        topic: &str,
        _queue: usize,
        callback: impl Fn(PoseArray) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
        subscribe_pose_array(self.node.clone(), topic, callback)
    }

    pub fn subscribe_path(
        &self,
        topic: &str,
        _queue: usize,
        callback: impl Fn(Path) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
        subscribe_path(self.node.clone(), topic, callback)
    }

    pub fn publish_twist(
        &self,
        topic: &str,
        _queue: usize,
    ) -> Result<TwistPublisher, Box<dyn std::error::Error>> {
        publish_twist(self.node.clone(), topic)
    }

    pub fn send_twist(pub_: &TwistPublisher, msg: Twist) {
        send_twist(pub_, msg)
    }

    pub fn publish_pose(
        &self,
        topic: &str,
        _queue: usize,
    ) -> Result<PosePublisher, Box<dyn std::error::Error>> {
        publish_pose(self.node.clone(), topic)
    }

    pub fn send_pose(pub_: &PosePublisher, pose: Pose) {
        send_pose(pub_, pose)
    }

    pub fn publish_pose_stamped(
        &self,
        topic: &str,
        _queue: usize,
    ) -> Result<PoseStampedPublisher, Box<dyn std::error::Error>> {
        publish_pose_stamped(self.node.clone(), topic)
    }

    pub fn send_pose_stamped(pub_: &PoseStampedPublisher, pose: Pose, frame_id: String) {
        send_pose_stamped(pub_, pose, frame_id)
    }

    pub fn publish_pose_with_cov_stamped(
        &self,
        topic: &str,
        _queue: usize,
    ) -> Result<PoseWithCovStampedPublisher, Box<dyn std::error::Error>> {
        publish_pose_with_cov_stamped(self.node.clone(), topic)
    }

    pub fn send_pose_with_cov_stamped(
        pub_: &PoseWithCovStampedPublisher,
        pose: Pose,
        frame_id: String,
    ) {
        send_pose_with_cov_stamped(pub_, pose, frame_id)
    }
}

impl Drop for Ros2Runtime {
    fn drop(&mut self) {
        self.spinner_running.store(false, Ordering::Relaxed);
        if let Some(spinner) = self.spinner.take() {
            let _ = spinner.join();
        }
    }
}

pub struct SubscriptionHandle {
    _join: tokio::task::JoinHandle<()>,
}

fn time_from_ros2(t: r2r::builtin_interfaces::msg::Time) -> Time {
    Time {
        sec: t.sec as i64,
        nsec: t.nanosec,
    }
}

fn duration_from_ros2(d: r2r::builtin_interfaces::msg::Duration) -> Duration {
    Duration {
        sec: d.sec as i64,
        nsec: d.nanosec,
    }
}

fn header_from_ros2(h: r2r::std_msgs::msg::Header) -> Header {
    Header {
        frame_id: h.frame_id,
        stamp: time_from_ros2(h.stamp),
    }
}

fn pose_from_ros2(p: r2r::geometry_msgs::msg::Pose) -> Pose {
    Pose {
        position: Point {
            x: p.position.x,
            y: p.position.y,
            z: p.position.z,
        },
        orientation: Quaternion {
            x: p.orientation.x,
            y: p.orientation.y,
            z: p.orientation.z,
            w: p.orientation.w,
        },
    }
}

fn transform_from_ros2(t: r2r::geometry_msgs::msg::Transform) -> Transform {
    Transform {
        translation: Vector3 {
            x: t.translation.x,
            y: t.translation.y,
            z: t.translation.z,
        },
        rotation: Quaternion {
            x: t.rotation.x,
            y: t.rotation.y,
            z: t.rotation.z,
            w: t.rotation.w,
        },
    }
}

fn isometry_from_transform(tf: &Transform) -> Isometry3<f64> {
    let tra = Translation3::new(tf.translation.x, tf.translation.y, tf.translation.z);
    let rot = UnitQuaternion::new_normalize(NaQuaternion::new(
        tf.rotation.w,
        tf.rotation.x,
        tf.rotation.y,
        tf.rotation.z,
    ));
    Isometry3::from_parts(tra, rot)
}

fn default_qos() -> r2r::QosProfile {
    // Use the default (volatile) QoS for most dynamic topics. Transient-local
    // durability is applied explicitly where needed (e.g. maps, /tf_static).
    r2r::QosProfile::default()
}

fn start_spinner(
    node: Arc<Mutex<r2r::Node>>,
    running: Arc<AtomicBool>,
) -> thread::JoinHandle<()> {
    thread::spawn(move || {
        while running.load(Ordering::Relaxed) {
            match node.lock() {
                Ok(mut node) => node.spin_once(StdDuration::from_millis(20)),
                Err(_) => break,
            }
        }
    })
}

pub struct TfBuffer {
    // stores transform that maps from `child` -> `parent`
    edges: RwLock<HashMap<(String, String), Isometry3<f64>>>,
}

impl TfBuffer {
    pub fn new() -> Self {
        Self {
            edges: RwLock::new(HashMap::new()),
        }
    }

    pub fn start(self: Arc<Self>, node: Arc<Mutex<r2r::Node>>) -> Result<(), Box<dyn std::error::Error>> {
        let mut node = node.lock().unwrap();
        let mut tf_sub = node.subscribe::<r2r::tf2_msgs::msg::TFMessage>(
            "/tf",
            r2r::QosProfile::default(),
        )?;
        let mut tf_static_sub = node.subscribe::<r2r::tf2_msgs::msg::TFMessage>(
            "/tf_static",
            r2r::QosProfile::transient_local(r2r::QosProfile::default()),
        )?;

        let buffer_tf = self.clone();
        tokio::spawn(async move {
            while let Some(msg) = tf_sub.next().await {
                buffer_tf.ingest_tf_message(msg);
            }
        });

        let buffer_tf_static = self.clone();
        tokio::spawn(async move {
            while let Some(msg) = tf_static_sub.next().await {
                buffer_tf_static.ingest_tf_message(msg);
            }
        });

        Ok(())
    }

    fn ingest_tf_message(&self, msg: r2r::tf2_msgs::msg::TFMessage) {
        let mut edges = self.edges.write().unwrap();
        for ts in msg.transforms {
            let parent = ts.header.frame_id;
            let child = ts.child_frame_id;
            let iso = isometry_from_transform(&transform_from_ros2(ts.transform));
            edges.insert((parent, child), iso);
        }
    }

    fn lookup_iso(&self, target: &str, source: &str) -> Option<Isometry3<f64>> {
        if target == source {
            return Some(Isometry3::identity());
        }

        let edges = self.edges.read().unwrap();
        let mut visited: HashMap<String, Isometry3<f64>> = HashMap::new();
        let mut queue: VecDeque<String> = VecDeque::new();

        visited.insert(source.to_string(), Isometry3::identity());
        queue.push_back(source.to_string());

        while let Some(current) = queue.pop_front() {
            let current_to_target = visited.get(&current).cloned().unwrap();

            // direct edge: parent <- child (child->parent)
            for ((parent, child), iso_child_to_parent) in edges.iter() {
                if child == &current {
                    // move current(child) -> parent
                    if !visited.contains_key(parent) {
                        visited.insert(parent.clone(), iso_child_to_parent * current_to_target);
                        queue.push_back(parent.clone());
                    }
                }
                if parent == &current {
                    // move current(parent) -> child (need inverse)
                    if !visited.contains_key(child) {
                        let inv = iso_child_to_parent.inverse();
                        visited.insert(child.clone(), inv * current_to_target);
                        queue.push_back(child.clone());
                    }
                }
            }

            if let Some(result) = visited.get(target) {
                return Some(*result);
            }
        }

        None
    }
}

impl TfClient for TfBuffer {
    fn lookup_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        _time: Time,
    ) -> Result<TransformStamped, TfError> {
        let iso = self
            .lookup_iso(target_frame, source_frame)
            .ok_or_else(|| TfError(format!("No transform {source_frame} -> {target_frame}")))?;

        // Convert isometry back to Transform
        let t = iso.translation.vector;
        let q = iso.rotation.quaternion();

        Ok(TransformStamped {
            parent_frame: target_frame.to_string(),
            child_frame: source_frame.to_string(),
            stamp: Time::zero(),
            transform: Transform {
                translation: Vector3 {
                    x: t.x,
                    y: t.y,
                    z: t.z,
                },
                rotation: Quaternion {
                    x: q.i,
                    y: q.j,
                    z: q.k,
                    w: q.w,
                },
            },
        })
    }
}

pub fn subscribe_laserscan(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
    callback: impl Fn(LaserScan) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    let mut sub = node.subscribe::<r2r::sensor_msgs::msg::LaserScan>(topic, default_qos())?;
    let join = tokio::spawn(async move {
        while let Some(scan) = sub.next().await {
            callback(LaserScan {
                header: header_from_ros2(scan.header),
                angle_min: scan.angle_min,
                angle_increment: scan.angle_increment,
                ranges: scan.ranges,
                range_min: scan.range_min,
            });
        }
    });
    Ok(SubscriptionHandle { _join: join })
}

pub fn subscribe_occupancy_grid(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
    callback: impl Fn(OccupancyGrid) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
    // Use default (volatile) QoS for bag playback compatibility. Transient-local
    // durability can be enabled explicitly when connecting to live latched map
    // publishers.
    let qos = default_qos();
    let mut node = node.lock().unwrap();
    let mut sub = node.subscribe::<r2r::nav_msgs::msg::OccupancyGrid>(topic, qos)?;
    let join = tokio::spawn(async move {
        while let Some(map) = sub.next().await {
            callback(OccupancyGrid {
                header: header_from_ros2(map.header),
                info: MapMetaData {
                    resolution: map.info.resolution,
                    width: map.info.width,
                    height: map.info.height,
                    origin: pose_from_ros2(map.info.origin),
                },
                data: map.data,
            });
        }
    });
    Ok(SubscriptionHandle { _join: join })
}

pub fn subscribe_pointcloud2(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
    callback: impl Fn(PointCloud2) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    let mut sub = node.subscribe::<r2r::sensor_msgs::msg::PointCloud2>(topic, default_qos())?;
    let join = tokio::spawn(async move {
        while let Some(cloud) = sub.next().await {
            callback(PointCloud2 {
                header: header_from_ros2(cloud.header),
                width: cloud.width,
                height: cloud.height,
                fields: cloud
                    .fields
                    .into_iter()
                    .map(|f| PointField {
                        name: f.name,
                        offset: f.offset,
                    })
                    .collect(),
                point_step: cloud.point_step,
                is_bigendian: cloud.is_bigendian,
                data: cloud.data,
            });
        }
    });
    Ok(SubscriptionHandle { _join: join })
}

pub fn subscribe_polygon_stamped(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
    callback: impl Fn(PolygonStamped) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    let mut sub = node.subscribe::<r2r::geometry_msgs::msg::PolygonStamped>(topic, default_qos())?;
    let join = tokio::spawn(async move {
        while let Some(msg) = sub.next().await {
            callback(PolygonStamped {
                header: header_from_ros2(msg.header),
                polygon: Polygon {
                    points: msg
                        .polygon
                        .points
                        .into_iter()
                        .map(|p| Point {
                            x: p.x as f64,
                            y: p.y as f64,
                            z: p.z as f64,
                        })
                        .collect(),
                },
            });
        }
    });
    Ok(SubscriptionHandle { _join: join })
}

pub fn subscribe_image(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
    callback: impl Fn(Image) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    let mut sub = node.subscribe::<r2r::sensor_msgs::msg::Image>(topic, default_qos())?;
    let join = tokio::spawn(async move {
        while let Some(img) = sub.next().await {
            callback(Image {
                header: header_from_ros2(img.header),
                width: img.width,
                height: img.height,
                encoding: img.encoding,
                is_bigendian: img.is_bigendian != 0,
                data: img.data,
            });
        }
    });
    Ok(SubscriptionHandle { _join: join })
}

fn marker_from_ros2(m: r2r::visualization_msgs::msg::Marker) -> Marker {
    Marker {
        header: header_from_ros2(m.header),
        ns: m.ns,
        id: m.id,
        type_: m.type_,
        action: m.action,
        pose: pose_from_ros2(m.pose),
        scale: Vector3 {
            x: m.scale.x,
            y: m.scale.y,
            z: m.scale.z,
        },
        color: ColorRGBA {
            r: m.color.r,
            g: m.color.g,
            b: m.color.b,
            a: m.color.a,
        },
        points: m
            .points
            .into_iter()
            .map(|p| Point {
                x: p.x,
                y: p.y,
                z: p.z,
            })
            .collect(),
        colors: m
            .colors
            .into_iter()
            .map(|c| ColorRGBA {
                r: c.r,
                g: c.g,
                b: c.b,
                a: c.a,
            })
            .collect(),
        lifetime: duration_from_ros2(m.lifetime),
    }
}

pub fn subscribe_marker(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
    callback: impl Fn(Marker) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    let mut sub = node.subscribe::<r2r::visualization_msgs::msg::Marker>(topic, default_qos())?;
    let join = tokio::spawn(async move {
        while let Some(m) = sub.next().await {
            callback(marker_from_ros2(m));
        }
    });
    Ok(SubscriptionHandle { _join: join })
}

pub fn subscribe_marker_array(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
    callback: impl Fn(MarkerArray) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    let mut sub = node.subscribe::<r2r::visualization_msgs::msg::MarkerArray>(topic, default_qos())?;
    let join = tokio::spawn(async move {
        while let Some(arr) = sub.next().await {
            callback(MarkerArray {
                markers: arr.markers.into_iter().map(marker_from_ros2).collect(),
            });
        }
    });
    Ok(SubscriptionHandle { _join: join })
}

pub fn subscribe_pose_stamped(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
    callback: impl Fn(PoseStamped) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    let mut sub = node.subscribe::<r2r::geometry_msgs::msg::PoseStamped>(topic, default_qos())?;
    let join = tokio::spawn(async move {
        while let Some(ps) = sub.next().await {
            callback(PoseStamped {
                header: header_from_ros2(ps.header),
                pose: pose_from_ros2(ps.pose),
            });
        }
    });
    Ok(SubscriptionHandle { _join: join })
}

pub fn subscribe_pose_array(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
    callback: impl Fn(PoseArray) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    let mut sub = node.subscribe::<r2r::geometry_msgs::msg::PoseArray>(topic, default_qos())?;
    let join = tokio::spawn(async move {
        while let Some(pa) = sub.next().await {
            callback(PoseArray {
                header: header_from_ros2(pa.header),
                poses: pa.poses.into_iter().map(pose_from_ros2).collect(),
            });
        }
    });
    Ok(SubscriptionHandle { _join: join })
}

pub fn subscribe_path(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
    callback: impl Fn(Path) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    let mut sub = node.subscribe::<r2r::nav_msgs::msg::Path>(topic, default_qos())?;
    let join = tokio::spawn(async move {
        while let Some(path) = sub.next().await {
            callback(Path {
                header: header_from_ros2(path.header),
                poses: path
                    .poses
                    .into_iter()
                    .map(|ps| PoseStamped {
                        header: header_from_ros2(ps.header),
                        pose: pose_from_ros2(ps.pose),
                    })
                    .collect(),
            });
        }
    });
    Ok(SubscriptionHandle { _join: join })
}

pub struct TwistPublisher {
    inner: r2r::Publisher<r2r::geometry_msgs::msg::Twist>,
}

pub fn publish_twist(node: Arc<Mutex<r2r::Node>>, topic: &str) -> Result<TwistPublisher, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    Ok(TwistPublisher {
        inner: node.create_publisher::<r2r::geometry_msgs::msg::Twist>(topic, r2r::QosProfile::default())?,
    })
}

pub fn send_twist(pub_: &TwistPublisher, msg: Twist) {
    let mut m = r2r::geometry_msgs::msg::Twist::default();
    m.linear.x = msg.linear_x;
    m.linear.y = msg.linear_y;
    m.angular.z = msg.angular_z;
    let _ = pub_.inner.publish(&m);
}

pub struct PosePublisher {
    inner: r2r::Publisher<r2r::geometry_msgs::msg::Pose>,
}

pub fn publish_pose(node: Arc<Mutex<r2r::Node>>, topic: &str) -> Result<PosePublisher, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    Ok(PosePublisher {
        inner: node.create_publisher::<r2r::geometry_msgs::msg::Pose>(topic, r2r::QosProfile::default())?,
    })
}

pub fn send_pose(pub_: &PosePublisher, pose: Pose) {
    let mut m = r2r::geometry_msgs::msg::Pose::default();
    m.position.x = pose.position.x;
    m.position.y = pose.position.y;
    m.position.z = pose.position.z;
    m.orientation.x = pose.orientation.x;
    m.orientation.y = pose.orientation.y;
    m.orientation.z = pose.orientation.z;
    m.orientation.w = pose.orientation.w;
    let _ = pub_.inner.publish(&m);
}

pub struct PoseStampedPublisher {
    inner: r2r::Publisher<r2r::geometry_msgs::msg::PoseStamped>,
}

pub fn publish_pose_stamped(node: Arc<Mutex<r2r::Node>>, topic: &str) -> Result<PoseStampedPublisher, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    Ok(PoseStampedPublisher {
        inner: node.create_publisher::<r2r::geometry_msgs::msg::PoseStamped>(topic, r2r::QosProfile::default())?,
    })
}

pub fn send_pose_stamped(pub_: &PoseStampedPublisher, pose: Pose, frame_id: String) {
    let mut m = r2r::geometry_msgs::msg::PoseStamped::default();
    m.header.frame_id = frame_id;
    m.pose = {
        let mut p = r2r::geometry_msgs::msg::Pose::default();
        p.position.x = pose.position.x;
        p.position.y = pose.position.y;
        p.position.z = pose.position.z;
        p.orientation.x = pose.orientation.x;
        p.orientation.y = pose.orientation.y;
        p.orientation.z = pose.orientation.z;
        p.orientation.w = pose.orientation.w;
        p
    };
    let _ = pub_.inner.publish(&m);
}

pub struct PoseWithCovStampedPublisher {
    inner: r2r::Publisher<r2r::geometry_msgs::msg::PoseWithCovarianceStamped>,
}

pub fn publish_pose_with_cov_stamped(
    node: Arc<Mutex<r2r::Node>>,
    topic: &str,
) -> Result<PoseWithCovStampedPublisher, Box<dyn std::error::Error>> {
    let mut node = node.lock().unwrap();
    Ok(PoseWithCovStampedPublisher {
        inner: node.create_publisher::<r2r::geometry_msgs::msg::PoseWithCovarianceStamped>(
            topic,
            r2r::QosProfile::default(),
        )?,
    })
}

pub fn send_pose_with_cov_stamped(
    pub_: &PoseWithCovStampedPublisher,
    pose: Pose,
    frame_id: String,
) {
    let mut m = r2r::geometry_msgs::msg::PoseWithCovarianceStamped::default();
    m.header.frame_id = frame_id;
    m.pose.pose = {
        let mut p = r2r::geometry_msgs::msg::Pose::default();
        p.position.x = pose.position.x;
        p.position.y = pose.position.y;
        p.position.z = pose.position.z;
        p.orientation.x = pose.orientation.x;
        p.orientation.y = pose.orientation.y;
        p.orientation.z = pose.orientation.z;
        p.orientation.w = pose.orientation.w;
        p
    };
    let _ = pub_.inner.publish(&m);
}
