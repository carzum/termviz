use crate::ros::tf::{TfClient, TfError};
use crate::ros::types::*;
use nalgebra::geometry::{Isometry3, Quaternion as NaQuaternion, Translation3, UnitQuaternion};
use serde::de::DeserializeOwned;
use std::error::Error;
use std::sync::Arc;

pub struct Ros1Runtime {
    tf: Arc<rustros_tf::TfListener>,
}

impl Ros1Runtime {
    pub fn init(name: &str) -> Result<Self, Box<dyn Error>> {
        rosrust::init(name);
        Ok(Self {
            tf: Arc::new(rustros_tf::TfListener::new()),
        })
    }

    pub fn tf_client(&self) -> Arc<dyn TfClient> {
        self.tf.clone()
    }

    pub fn now() -> Time {
        let t = rosrust::Time::new();
        Time {
            sec: t.sec as i64,
            nsec: t.nsec,
        }
    }

    pub fn topics(&self) -> Vec<(String, String)> {
        match rosrust::topics() {
            Ok(list) => list
                .into_iter()
                .map(|t| (t.name.to_string(), t.datatype.to_string()))
                .collect(),
            Err(_) => Vec::new(),
        }
    }

    pub fn param_get<T: DeserializeOwned>(&self, name: &str) -> Option<T> {
        rosrust::param(name).and_then(|p| p.get::<T>().ok())
    }

    pub fn subscribe_laserscan(
        &self,
        topic: &str,
        queue: usize,
        callback: impl Fn(LaserScan) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn Error>> {
        Ok(subscribe_laserscan(topic, queue, callback))
    }

    pub fn subscribe_occupancy_grid(
        &self,
        topic: &str,
        queue: usize,
        callback: impl Fn(OccupancyGrid) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn Error>> {
        Ok(subscribe_occupancy_grid(topic, queue, callback))
    }

    pub fn subscribe_pointcloud2(
        &self,
        topic: &str,
        queue: usize,
        callback: impl Fn(PointCloud2) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn Error>> {
        Ok(subscribe_pointcloud2(topic, queue, callback))
    }

    pub fn subscribe_polygon_stamped(
        &self,
        topic: &str,
        queue: usize,
        callback: impl Fn(PolygonStamped) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn Error>> {
        Ok(subscribe_polygon_stamped(topic, queue, callback))
    }

    pub fn subscribe_image(
        &self,
        topic: &str,
        queue: usize,
        callback: impl Fn(Image) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn Error>> {
        Ok(subscribe_image(topic, queue, callback))
    }

    pub fn subscribe_marker(
        &self,
        topic: &str,
        queue: usize,
        callback: impl Fn(Marker) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn Error>> {
        Ok(subscribe_marker(topic, queue, callback))
    }

    pub fn subscribe_marker_array(
        &self,
        topic: &str,
        queue: usize,
        callback: impl Fn(MarkerArray) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn Error>> {
        Ok(subscribe_marker_array(topic, queue, callback))
    }

    pub fn subscribe_pose_stamped(
        &self,
        topic: &str,
        queue: usize,
        callback: impl Fn(PoseStamped) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn Error>> {
        Ok(subscribe_pose_stamped(topic, queue, callback))
    }

    pub fn subscribe_pose_array(
        &self,
        topic: &str,
        queue: usize,
        callback: impl Fn(PoseArray) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn Error>> {
        Ok(subscribe_pose_array(topic, queue, callback))
    }

    pub fn subscribe_path(
        &self,
        topic: &str,
        queue: usize,
        callback: impl Fn(Path) + Send + 'static,
    ) -> Result<SubscriptionHandle, Box<dyn Error>> {
        Ok(subscribe_path(topic, queue, callback))
    }

    pub fn publish_twist(&self, topic: &str, queue: usize) -> Result<TwistPublisher, Box<dyn Error>> {
        Ok(publish_twist(topic, queue))
    }

    pub fn send_twist(pub_: &TwistPublisher, msg: Twist) {
        send_twist(pub_, msg)
    }

    pub fn publish_pose(&self, topic: &str, queue: usize) -> Result<PosePublisher, Box<dyn Error>> {
        Ok(publish_pose(topic, queue))
    }

    pub fn send_pose(pub_: &PosePublisher, pose: Pose) {
        send_pose(pub_, pose)
    }

    pub fn publish_pose_stamped(
        &self,
        topic: &str,
        queue: usize,
    ) -> Result<PoseStampedPublisher, Box<dyn Error>> {
        Ok(publish_pose_stamped(topic, queue))
    }

    pub fn send_pose_stamped(pub_: &PoseStampedPublisher, pose: Pose, frame_id: String) {
        send_pose_stamped(pub_, pose, frame_id)
    }

    pub fn publish_pose_with_cov_stamped(
        &self,
        topic: &str,
        queue: usize,
    ) -> Result<PoseWithCovStampedPublisher, Box<dyn Error>> {
        Ok(publish_pose_with_cov_stamped(topic, queue))
    }

    pub fn send_pose_with_cov_stamped(
        pub_: &PoseWithCovStampedPublisher,
        pose: Pose,
        frame_id: String,
    ) {
        send_pose_with_cov_stamped(pub_, pose, frame_id)
    }
}

impl TfClient for rustros_tf::TfListener {
    fn lookup_transform(
        &self,
        target_frame: &str,
        source_frame: &str,
        time: Time,
    ) -> Result<TransformStamped, TfError> {
        let t = rosrust::Time {
            sec: time.sec as u32,
            nsec: time.nsec,
        };
        self.lookup_transform(target_frame, source_frame, t)
            .map(|ts| TransformStamped {
                parent_frame: ts.header.frame_id,
                child_frame: ts.child_frame_id,
                stamp: Time {
                    sec: ts.header.stamp.sec as i64,
                    nsec: ts.header.stamp.nsec,
                },
                transform: Transform {
                    translation: Vector3 {
                        x: ts.transform.translation.x,
                        y: ts.transform.translation.y,
                        z: ts.transform.translation.z,
                    },
                    rotation: Quaternion {
                        x: ts.transform.rotation.x,
                        y: ts.transform.rotation.y,
                        z: ts.transform.rotation.z,
                        w: ts.transform.rotation.w,
                    },
                },
            })
            .map_err(|e| TfError(format!("{e:?}")))
    }
}

#[allow(dead_code)]
pub fn isometry_from_transform(tf: &Transform) -> Isometry3<f64> {
    let tra = Translation3::new(tf.translation.x, tf.translation.y, tf.translation.z);
    let rot = UnitQuaternion::new_normalize(NaQuaternion::new(
        tf.rotation.w,
        tf.rotation.x,
        tf.rotation.y,
        tf.rotation.z,
    ));
    Isometry3::from_parts(tra, rot)
}

#[allow(dead_code)]
pub fn isometry_from_pose(pose: &Pose) -> Isometry3<f64> {
    let tra = Translation3::new(pose.position.x, pose.position.y, pose.position.z);
    let rot = UnitQuaternion::new_normalize(NaQuaternion::new(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
    ));
    Isometry3::from_parts(tra, rot)
}

pub type SubscriptionHandle = rosrust::Subscriber;

pub fn subscribe_laserscan(
    topic: &str,
    queue: usize,
    callback: impl Fn(LaserScan) + Send + 'static,
) -> SubscriptionHandle {
    rosrust::subscribe(topic, queue, move |scan: rosrust_msg::sensor_msgs::LaserScan| {
        callback(LaserScan {
            header: Header {
                frame_id: scan.header.frame_id,
                stamp: Time {
                    sec: scan.header.stamp.sec as i64,
                    nsec: scan.header.stamp.nsec,
                },
            },
            angle_min: scan.angle_min,
            angle_increment: scan.angle_increment,
            ranges: scan.ranges,
            range_min: scan.range_min,
        })
    })
    .unwrap()
}

pub fn subscribe_occupancy_grid(
    topic: &str,
    queue: usize,
    callback: impl Fn(OccupancyGrid) + Send + 'static,
) -> SubscriptionHandle {
    rosrust::subscribe(topic, queue, move |map: rosrust_msg::nav_msgs::OccupancyGrid| {
        callback(OccupancyGrid {
            header: Header {
                frame_id: map.header.frame_id,
                stamp: Time {
                    sec: map.header.stamp.sec as i64,
                    nsec: map.header.stamp.nsec,
                },
            },
            info: MapMetaData {
                resolution: map.info.resolution,
                width: map.info.width,
                height: map.info.height,
                origin: Pose {
                    position: Point {
                        x: map.info.origin.position.x,
                        y: map.info.origin.position.y,
                        z: map.info.origin.position.z,
                    },
                    orientation: Quaternion {
                        x: map.info.origin.orientation.x,
                        y: map.info.origin.orientation.y,
                        z: map.info.origin.orientation.z,
                        w: map.info.origin.orientation.w,
                    },
                },
            },
            data: map.data,
        })
    })
    .unwrap()
}

pub fn subscribe_pointcloud2(
    topic: &str,
    queue: usize,
    callback: impl Fn(PointCloud2) + Send + 'static,
) -> SubscriptionHandle {
    rosrust::subscribe(
        topic,
        queue,
        move |cloud: rosrust_msg::sensor_msgs::PointCloud2| {
            callback(PointCloud2 {
                header: Header {
                    frame_id: cloud.header.frame_id,
                    stamp: Time {
                        sec: cloud.header.stamp.sec as i64,
                        nsec: cloud.header.stamp.nsec,
                    },
                },
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
            })
        },
    )
    .unwrap()
}

pub fn subscribe_polygon_stamped(
    topic: &str,
    queue: usize,
    callback: impl Fn(PolygonStamped) + Send + 'static,
) -> SubscriptionHandle {
    rosrust::subscribe(
        topic,
        queue,
        move |msg: rosrust_msg::geometry_msgs::PolygonStamped| {
            callback(PolygonStamped {
                header: Header {
                    frame_id: msg.header.frame_id,
                    stamp: Time {
                        sec: msg.header.stamp.sec as i64,
                        nsec: msg.header.stamp.nsec,
                    },
                },
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
            })
        },
    )
    .unwrap()
}

pub fn subscribe_image(
    topic: &str,
    queue: usize,
    callback: impl Fn(Image) + Send + 'static,
) -> SubscriptionHandle {
    rosrust::subscribe(topic, queue, move |img: rosrust_msg::sensor_msgs::Image| {
        callback(Image {
            header: Header {
                frame_id: img.header.frame_id,
                stamp: Time {
                    sec: img.header.stamp.sec as i64,
                    nsec: img.header.stamp.nsec,
                },
            },
            width: img.width,
            height: img.height,
            encoding: img.encoding,
            is_bigendian: img.is_bigendian != 0,
            data: img.data,
        })
    })
    .unwrap()
}

pub fn subscribe_marker(
    topic: &str,
    queue: usize,
    callback: impl Fn(Marker) + Send + 'static,
) -> SubscriptionHandle {
    rosrust::subscribe(topic, queue, move |msg: rosrust_msg::visualization_msgs::Marker| {
        callback(convert_marker(msg))
    })
    .unwrap()
}

pub fn subscribe_marker_array(
    topic: &str,
    queue: usize,
    callback: impl Fn(MarkerArray) + Send + 'static,
) -> SubscriptionHandle {
    rosrust::subscribe(
        topic,
        queue,
        move |msg: rosrust_msg::visualization_msgs::MarkerArray| {
            callback(MarkerArray {
                markers: msg.markers.into_iter().map(convert_marker).collect(),
            })
        },
    )
    .unwrap()
}

fn convert_marker(msg: rosrust_msg::visualization_msgs::Marker) -> Marker {
    Marker {
        header: Header {
            frame_id: msg.header.frame_id,
            stamp: Time {
                sec: msg.header.stamp.sec as i64,
                nsec: msg.header.stamp.nsec,
            },
        },
        ns: msg.ns,
        id: msg.id,
        type_: msg.type_,
        action: msg.action,
        pose: Pose {
            position: Point {
                x: msg.pose.position.x,
                y: msg.pose.position.y,
                z: msg.pose.position.z,
            },
            orientation: Quaternion {
                x: msg.pose.orientation.x,
                y: msg.pose.orientation.y,
                z: msg.pose.orientation.z,
                w: msg.pose.orientation.w,
            },
        },
        scale: Vector3 {
            x: msg.scale.x,
            y: msg.scale.y,
            z: msg.scale.z,
        },
        color: ColorRGBA {
            r: msg.color.r,
            g: msg.color.g,
            b: msg.color.b,
            a: msg.color.a,
        },
        points: msg
            .points
            .into_iter()
            .map(|p| Point {
                x: p.x,
                y: p.y,
                z: p.z,
            })
            .collect(),
        colors: msg
            .colors
            .into_iter()
            .map(|c| ColorRGBA {
                r: c.r,
                g: c.g,
                b: c.b,
                a: c.a,
            })
            .collect(),
        lifetime: Duration {
            sec: msg.lifetime.sec as i64,
            nsec: msg.lifetime.nsec as u32,
        },
    }
}

pub fn subscribe_pose_stamped(
    topic: &str,
    queue: usize,
    callback: impl Fn(PoseStamped) + Send + 'static,
) -> SubscriptionHandle {
    rosrust::subscribe(
        topic,
        queue,
        move |msg: rosrust_msg::geometry_msgs::PoseStamped| {
            callback(PoseStamped {
                header: Header {
                    frame_id: msg.header.frame_id,
                    stamp: Time {
                        sec: msg.header.stamp.sec as i64,
                        nsec: msg.header.stamp.nsec,
                    },
                },
                pose: Pose {
                    position: Point {
                        x: msg.pose.position.x,
                        y: msg.pose.position.y,
                        z: msg.pose.position.z,
                    },
                    orientation: Quaternion {
                        x: msg.pose.orientation.x,
                        y: msg.pose.orientation.y,
                        z: msg.pose.orientation.z,
                        w: msg.pose.orientation.w,
                    },
                },
            })
        },
    )
    .unwrap()
}

pub fn subscribe_pose_array(
    topic: &str,
    queue: usize,
    callback: impl Fn(PoseArray) + Send + 'static,
) -> SubscriptionHandle {
    rosrust::subscribe(
        topic,
        queue,
        move |msg: rosrust_msg::geometry_msgs::PoseArray| {
            callback(PoseArray {
                header: Header {
                    frame_id: msg.header.frame_id,
                    stamp: Time {
                        sec: msg.header.stamp.sec as i64,
                        nsec: msg.header.stamp.nsec,
                    },
                },
                poses: msg
                    .poses
                    .into_iter()
                    .map(|p| Pose {
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
                    })
                    .collect(),
            })
        },
    )
    .unwrap()
}

pub fn subscribe_path(
    topic: &str,
    queue: usize,
    callback: impl Fn(Path) + Send + 'static,
) -> SubscriptionHandle {
    rosrust::subscribe(topic, queue, move |msg: rosrust_msg::nav_msgs::Path| {
        callback(Path {
            header: Header {
                frame_id: msg.header.frame_id,
                stamp: Time {
                    sec: msg.header.stamp.sec as i64,
                    nsec: msg.header.stamp.nsec,
                },
            },
            poses: msg
                .poses
                .into_iter()
                .map(|ps| PoseStamped {
                    header: Header {
                        frame_id: ps.header.frame_id,
                        stamp: Time {
                            sec: ps.header.stamp.sec as i64,
                            nsec: ps.header.stamp.nsec,
                        },
                    },
                    pose: Pose {
                        position: Point {
                            x: ps.pose.position.x,
                            y: ps.pose.position.y,
                            z: ps.pose.position.z,
                        },
                        orientation: Quaternion {
                            x: ps.pose.orientation.x,
                            y: ps.pose.orientation.y,
                            z: ps.pose.orientation.z,
                            w: ps.pose.orientation.w,
                        },
                    },
                })
                .collect(),
        })
    })
    .unwrap()
}

pub type TwistPublisher = rosrust::Publisher<rosrust_msg::geometry_msgs::Twist>;

pub fn publish_twist(topic: &str, queue: usize) -> TwistPublisher {
    rosrust::publish(topic, queue).unwrap()
}

pub fn send_twist(pub_: &TwistPublisher, msg: Twist) {
    let mut m = rosrust_msg::geometry_msgs::Twist::default();
    m.linear.x = msg.linear_x;
    m.linear.y = msg.linear_y;
    m.angular.z = msg.angular_z;
    pub_.send(m).unwrap();
}

pub type PosePublisher = rosrust::Publisher<rosrust_msg::geometry_msgs::Pose>;

pub fn publish_pose(topic: &str, queue: usize) -> PosePublisher {
    rosrust::publish(topic, queue).unwrap()
}

pub fn send_pose(pub_: &PosePublisher, pose: Pose) {
    let mut m = rosrust_msg::geometry_msgs::Pose::default();
    m.position.x = pose.position.x;
    m.position.y = pose.position.y;
    m.position.z = pose.position.z;
    m.orientation.x = pose.orientation.x;
    m.orientation.y = pose.orientation.y;
    m.orientation.z = pose.orientation.z;
    m.orientation.w = pose.orientation.w;
    pub_.send(m).unwrap();
}

pub type PoseStampedPublisher = rosrust::Publisher<rosrust_msg::geometry_msgs::PoseStamped>;

pub fn publish_pose_stamped(topic: &str, queue: usize) -> PoseStampedPublisher {
    rosrust::publish(topic, queue).unwrap()
}

pub fn send_pose_stamped(pub_: &PoseStampedPublisher, pose: Pose, frame_id: String) {
    let mut m = rosrust_msg::geometry_msgs::PoseStamped::default();
    m.header.frame_id = frame_id;
    m.pose.position.x = pose.position.x;
    m.pose.position.y = pose.position.y;
    m.pose.position.z = pose.position.z;
    m.pose.orientation.x = pose.orientation.x;
    m.pose.orientation.y = pose.orientation.y;
    m.pose.orientation.z = pose.orientation.z;
    m.pose.orientation.w = pose.orientation.w;
    pub_.send(m).unwrap();
}

pub type PoseWithCovStampedPublisher =
    rosrust::Publisher<rosrust_msg::geometry_msgs::PoseWithCovarianceStamped>;

pub fn publish_pose_with_cov_stamped(topic: &str, queue: usize) -> PoseWithCovStampedPublisher {
    rosrust::publish(topic, queue).unwrap()
}

pub fn send_pose_with_cov_stamped(
    pub_: &PoseWithCovStampedPublisher,
    pose: Pose,
    frame_id: String,
) {
    let mut m = rosrust_msg::geometry_msgs::PoseWithCovarianceStamped::default();
    m.header.frame_id = frame_id;
    m.pose.pose.position.x = pose.position.x;
    m.pose.pose.position.y = pose.position.y;
    m.pose.pose.position.z = pose.position.z;
    m.pose.pose.orientation.x = pose.orientation.x;
    m.pose.pose.orientation.y = pose.orientation.y;
    m.pose.pose.orientation.z = pose.orientation.z;
    m.pose.pose.orientation.w = pose.orientation.w;
    pub_.send(m).unwrap();
}
