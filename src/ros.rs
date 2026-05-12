#[cfg(all(feature = "ros1", feature = "ros2"))]
compile_error!("Features 'ros1' and 'ros2' are mutually exclusive");

#[cfg(not(any(feature = "ros1", feature = "ros2")))]
compile_error!("Enable exactly one backend feature: 'ros1' or 'ros2'");

pub mod tf;
pub mod types;

#[cfg(feature = "ros1")]
pub mod ros1;

#[cfg(feature = "ros2")]
pub mod ros2;

use serde::de::DeserializeOwned;
use std::cell::RefCell;
use std::error::Error;
use std::sync::Arc;

#[cfg(feature = "ros1")]
type ActiveRuntime = ros1::Ros1Runtime;

#[cfg(feature = "ros2")]
type ActiveRuntime = ros2::Ros2Runtime;

thread_local! {
    static RUNTIME: RefCell<Option<ActiveRuntime>> = RefCell::new(None);
}

pub fn init(name: &str) -> Result<(), Box<dyn Error>> {
    let runtime = ActiveRuntime::init(name)?;
    RUNTIME.with(|cell| {
        let mut slot = cell.borrow_mut();
        if slot.is_none() {
            *slot = Some(runtime);
        }
    });
    Ok(())
}

fn with_runtime<T>(f: impl FnOnce(&ActiveRuntime) -> T) -> T {
    RUNTIME.with(|cell| {
        let slot = cell.borrow();
        let runtime = slot.as_ref().expect("ros::init() must be called first");
        f(runtime)
    })
}

pub fn now() -> types::Time {
    ActiveRuntime::now()
}

pub fn tf_client() -> Arc<dyn tf::TfClient> {
    with_runtime(|rt| rt.tf_client())
}

pub fn topics() -> Vec<(String, String)> {
    with_runtime(|rt| rt.topics())
}

pub fn param_get<T: DeserializeOwned>(name: &str) -> Option<T> {
    with_runtime(|rt| rt.param_get(name))
}

#[cfg(feature = "ros1")]
pub type SubscriptionHandle = ros1::SubscriptionHandle;

#[cfg(feature = "ros2")]
pub type SubscriptionHandle = ros2::SubscriptionHandle;

pub fn subscribe_laserscan(
    topic: &str,
    queue: usize,
    callback: impl Fn(types::LaserScan) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn Error>> {
    with_runtime(|rt| rt.subscribe_laserscan(topic, queue, callback))
}

pub fn subscribe_occupancy_grid(
    topic: &str,
    queue: usize,
    callback: impl Fn(types::OccupancyGrid) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn Error>> {
    with_runtime(|rt| rt.subscribe_occupancy_grid(topic, queue, callback))
}

pub fn subscribe_pointcloud2(
    topic: &str,
    queue: usize,
    callback: impl Fn(types::PointCloud2) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn Error>> {
    with_runtime(|rt| rt.subscribe_pointcloud2(topic, queue, callback))
}

pub fn subscribe_polygon_stamped(
    topic: &str,
    queue: usize,
    callback: impl Fn(types::PolygonStamped) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn Error>> {
    with_runtime(|rt| rt.subscribe_polygon_stamped(topic, queue, callback))
}

pub fn subscribe_image(
    topic: &str,
    queue: usize,
    callback: impl Fn(types::Image) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn Error>> {
    with_runtime(|rt| rt.subscribe_image(topic, queue, callback))
}

pub fn subscribe_marker(
    topic: &str,
    queue: usize,
    callback: impl Fn(types::Marker) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn Error>> {
    with_runtime(|rt| rt.subscribe_marker(topic, queue, callback))
}

pub fn subscribe_marker_array(
    topic: &str,
    queue: usize,
    callback: impl Fn(types::MarkerArray) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn Error>> {
    with_runtime(|rt| rt.subscribe_marker_array(topic, queue, callback))
}

pub fn subscribe_pose_stamped(
    topic: &str,
    queue: usize,
    callback: impl Fn(types::PoseStamped) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn Error>> {
    with_runtime(|rt| rt.subscribe_pose_stamped(topic, queue, callback))
}

pub fn subscribe_pose_array(
    topic: &str,
    queue: usize,
    callback: impl Fn(types::PoseArray) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn Error>> {
    with_runtime(|rt| rt.subscribe_pose_array(topic, queue, callback))
}

pub fn subscribe_path(
    topic: &str,
    queue: usize,
    callback: impl Fn(types::Path) + Send + 'static,
) -> Result<SubscriptionHandle, Box<dyn Error>> {
    with_runtime(|rt| rt.subscribe_path(topic, queue, callback))
}

#[cfg(feature = "ros1")]
pub type TwistPublisher = ros1::TwistPublisher;

#[cfg(feature = "ros2")]
pub type TwistPublisher = ros2::TwistPublisher;

pub fn publish_twist(topic: &str, queue: usize) -> Result<TwistPublisher, Box<dyn Error>> {
    with_runtime(|rt| rt.publish_twist(topic, queue))
}

pub fn send_twist(pub_: &TwistPublisher, msg: types::Twist) {
    ActiveRuntime::send_twist(pub_, msg)
}

#[cfg(feature = "ros1")]
pub type PosePublisher = ros1::PosePublisher;

#[cfg(feature = "ros2")]
pub type PosePublisher = ros2::PosePublisher;

pub fn publish_pose(topic: &str, queue: usize) -> Result<PosePublisher, Box<dyn Error>> {
    with_runtime(|rt| rt.publish_pose(topic, queue))
}

pub fn send_pose(pub_: &PosePublisher, pose: types::Pose) {
    ActiveRuntime::send_pose(pub_, pose)
}

#[cfg(feature = "ros1")]
pub type PoseStampedPublisher = ros1::PoseStampedPublisher;

#[cfg(feature = "ros2")]
pub type PoseStampedPublisher = ros2::PoseStampedPublisher;

pub fn publish_pose_stamped(
    topic: &str,
    queue: usize,
) -> Result<PoseStampedPublisher, Box<dyn Error>> {
    with_runtime(|rt| rt.publish_pose_stamped(topic, queue))
}

pub fn send_pose_stamped(pub_: &PoseStampedPublisher, pose: types::Pose, frame_id: String) {
    ActiveRuntime::send_pose_stamped(pub_, pose, frame_id)
}

#[cfg(feature = "ros1")]
pub type PoseWithCovStampedPublisher = ros1::PoseWithCovStampedPublisher;

#[cfg(feature = "ros2")]
pub type PoseWithCovStampedPublisher = ros2::PoseWithCovStampedPublisher;

pub fn publish_pose_with_cov_stamped(
    topic: &str,
    queue: usize,
) -> Result<PoseWithCovStampedPublisher, Box<dyn Error>> {
    with_runtime(|rt| rt.publish_pose_with_cov_stamped(topic, queue))
}

pub fn send_pose_with_cov_stamped(
    pub_: &PoseWithCovStampedPublisher,
    pose: types::Pose,
    frame_id: String,
) {
    ActiveRuntime::send_pose_with_cov_stamped(pub_, pose, frame_id)
}
