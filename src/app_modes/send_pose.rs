//! Send pose mode allows to send a pose on the given topic.

use crate::app_modes::viewport::{UseViewport, Viewport};
use crate::app_modes::{input, AppMode, BaseMode};
use crate::config::SendPoseConfig;
use crate::footprint::get_current_footprint;
use crate::transformation;
use approx::AbsDiffEq;
use nalgebra::{Isometry2, Vector2};
use std::cell::RefCell;
use std::rc::Rc;
use tui::backend::Backend;
use tui::style::Color;
use tui::widgets::canvas::{Context, Line};

trait BasePosePubWrapper {
    fn get_topic(&self) -> &String;
    fn send(&self, msg: rosrust_msg::geometry_msgs::Pose, frame_id: String);
}

struct PosePubWrapper {
    topic: String,
    publisher: rosrust::Publisher<rosrust_msg::geometry_msgs::Pose>,
}

impl PosePubWrapper {
    pub fn new(topic: &String) -> PosePubWrapper {
        PosePubWrapper {
            topic: topic.clone(),
            publisher: rosrust::publish(&topic, 1).unwrap(),
        }
    }
}

impl BasePosePubWrapper for PosePubWrapper {
    fn get_topic(&self) -> &String {
        &self.topic
    }

    fn send(&self, msg: rosrust_msg::geometry_msgs::Pose, _frame_id: String) {
        self.publisher.send(msg).unwrap();
    }
}

struct PoseStampedPubWrapper {
    topic: String,
    publisher: rosrust::Publisher<rosrust_msg::geometry_msgs::PoseStamped>,
}

impl PoseStampedPubWrapper {
    pub fn new(topic: &String) -> PoseStampedPubWrapper {
        PoseStampedPubWrapper {
            topic: topic.clone(),
            publisher: rosrust::publish(&topic, 1).unwrap(),
        }
    }
}

impl BasePosePubWrapper for PoseStampedPubWrapper {
    fn get_topic(&self) -> &String {
        &self.topic
    }

    fn send(&self, msg: rosrust_msg::geometry_msgs::Pose, frame_id: String) {
        let mut msg_stamped = rosrust_msg::geometry_msgs::PoseStamped::default();
        msg_stamped.header.frame_id = frame_id;
        msg_stamped.pose.orientation.x = msg.orientation.x;
        msg_stamped.pose.orientation.y = msg.orientation.y;
        msg_stamped.pose.orientation.z = msg.orientation.z;
        msg_stamped.pose.orientation.w = msg.orientation.w;
        msg_stamped.pose.position.x = msg.position.x;
        msg_stamped.pose.position.y = msg.position.y;
        msg_stamped.pose.position.z = 0.0;
        self.publisher.send(msg_stamped).unwrap();
    }
}

struct PoseCovPubWrapper {
    topic: String,
    publisher: rosrust::Publisher<rosrust_msg::geometry_msgs::PoseWithCovarianceStamped>,
}

impl PoseCovPubWrapper {
    pub fn new(topic: &String) -> PoseCovPubWrapper {
        PoseCovPubWrapper {
            topic: topic.clone(),
            publisher: rosrust::publish(&topic, 1).unwrap(),
        }
    }
}

impl BasePosePubWrapper for PoseCovPubWrapper {
    fn get_topic(&self) -> &String {
        &self.topic
    }

    fn send(&self, msg: rosrust_msg::geometry_msgs::Pose, frame_id: String) {
        let mut msg_cov = rosrust_msg::geometry_msgs::PoseWithCovarianceStamped::default();
        msg_cov.header.frame_id = frame_id;
        msg_cov.pose.pose.orientation.x = msg.orientation.x;
        msg_cov.pose.pose.orientation.y = msg.orientation.y;
        msg_cov.pose.pose.orientation.z = msg.orientation.z;
        msg_cov.pose.pose.orientation.w = msg.orientation.w;
        msg_cov.pose.pose.position.x = msg.position.x;
        msg_cov.pose.pose.position.y = msg.position.y;
        msg_cov.pose.pose.position.z = 0.0;
        self.publisher.send(msg_cov).unwrap();
    }
}

/// Represents the send pose mode.
pub struct SendPose {
    viewport: Rc<RefCell<Viewport>>,
    increment: f64,
    robot_pose: Isometry2<f64>,
    new_pose: Isometry2<f64>,
    current_topic: usize,
    publishers: Vec<Box<dyn BasePosePubWrapper>>,
    ghost_active: bool,
}

impl SendPose {
    pub fn new(topics: &Vec<SendPoseConfig>, viewport: Rc<RefCell<Viewport>>) -> SendPose {
        let base_link_pose = viewport
            .borrow()
            .tf_listener
            .lookup_transform(
                &viewport.borrow().static_frame,
                &viewport.borrow().robot_frame,
                rosrust::Time::new(),
            )
            .unwrap()
            .transform;
        let robot_pose = transformation::ros_to_iso2d(&base_link_pose);

        let mut publishers = Vec::<Box<dyn BasePosePubWrapper>>::new();

        for topic in topics {
            match topic.msg_type.as_str() {
                "Pose" => publishers.push(Box::new(PosePubWrapper::new(&topic.topic))),
                "PoseStamped" => {
                    publishers.push(Box::new(PoseStampedPubWrapper::new(&topic.topic)))
                }
                "PoseWithCovarianceStamped" => {
                    publishers.push(Box::new(PoseCovPubWrapper::new(&topic.topic)))
                }
                _ => {}
            }
        }

        SendPose {
            viewport: viewport,
            current_topic: 0,
            publishers: publishers,
            increment: 0.1,
            robot_pose: robot_pose.clone(),
            new_pose: robot_pose,
            ghost_active: false,
        }
    }

    fn move_new_pose(&mut self, x: f64, y: f64, yaw: f64) {
        let new_yaw = self.new_pose.rotation.angle() + yaw;
        let new_x = x * new_yaw.cos() - y * new_yaw.sin() + self.new_pose.translation.x;
        let new_y = x * new_yaw.sin() + y * new_yaw.cos() + self.new_pose.translation.y;
        self.new_pose = Isometry2::new(Vector2::new(new_x, new_y), new_yaw);
        self.ghost_active = true;
    }

    fn send_new_pose(&mut self) {
        if self.new_pose.abs_diff_ne(&self.robot_pose, 0.01) {
            let pose = transformation::iso2d_to_ros(&self.new_pose);
            let frame_id = self.viewport.borrow().static_frame.to_string();
            let mut msg = rosrust_msg::geometry_msgs::Pose::default();
            msg.orientation.x = pose.rotation.x;
            msg.orientation.y = pose.rotation.y;
            msg.orientation.z = pose.rotation.z;
            msg.orientation.w = pose.rotation.w;
            msg.position.x = pose.translation.x;
            msg.position.y = pose.translation.y;
            msg.position.z = 0.0;
            self.publishers[self.current_topic].send(msg, frame_id);
            self.ghost_active = false;
        }
    }
}

impl<B: Backend> BaseMode<B> for SendPose {}

impl AppMode for SendPose {
    fn run(&mut self) {
        let base_link_pose = self
            .viewport
            .borrow()
            .tf_listener
            .lookup_transform(
                &self.viewport.borrow().static_frame,
                &self.viewport.borrow().robot_frame,
                rosrust::Time::new(),
            )
            .unwrap()
            .transform;
        self.robot_pose = transformation::ros_to_iso2d(&base_link_pose);
        if !self.ghost_active {
            self.new_pose = self.robot_pose.clone();
        }
    }
    fn reset(&mut self) {
        self.ghost_active = false;
        self.run(); // Update the robot pose
    }
    fn handle_input(&mut self, input: &String) {
        self.viewport.borrow_mut().handle_input(input);
        match input.as_str() {
            input::UP => self.move_new_pose(self.increment, 0.0, 0.0),
            input::DOWN => self.move_new_pose(-self.increment, 0.0, 0.0),
            input::LEFT => self.move_new_pose(0.0, self.increment, 0.0),
            input::RIGHT => self.move_new_pose(0.0, -self.increment, 0.0),
            input::ROTATE_LEFT => self.move_new_pose(0.0, 0.0, self.increment),
            input::ROTATE_RIGHT => self.move_new_pose(0.0, 0.0, -self.increment),
            input::INCREMENT_STEP => self.increment += 0.1,
            input::DECREMENT_STEP => self.increment -= 0.1,
            input::NEXT => self.current_topic = (self.current_topic + 1) % self.publishers.len(),
            input::PREVIOUS => {
                self.current_topic = if self.current_topic > 0 {
                    self.current_topic - 1
                } else {
                    self.publishers.len() - 1
                };
            }
            input::CANCEL => self.reset(),
            input::CONFIRM => self.send_new_pose(),
            _ => (),
        }
    }

    fn get_name(&self) -> String {
        "Send Pose".to_string()
    }

    fn get_description(&self) -> Vec<String> {
        vec![
            "This mode allows to publish a pose message on a topic.".to_string(),
            "The top bar shows the current selected topic to which the pose is sent.".to_string(),
            "The viewport is centered on the preview outline of where the pose is on the map."
                .to_string(),
        ]
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        let mut keymap = vec![
            [
                input::UP.to_string(),
                "Shifts the desired pose positively along the x axis.".to_string(),
            ],
            [
                input::DOWN.to_string(),
                "Shifts the desired pose negatively along the x axis.".to_string(),
            ],
            [
                input::RIGHT.to_string(),
                "Shifts the desired pose positively along the y axis.".to_string(),
            ],
            [
                input::LEFT.to_string(),
                "Shifts the desired pose negatively along the y axis.".to_string(),
            ],
            [
                input::ROTATE_LEFT.to_string(),
                "Rotates the desired pose counter-clockwise.".to_string(),
            ],
            [
                input::ROTATE_RIGHT.to_string(),
                "Rotates the desired pose clockwise.".to_string(),
            ],
            [
                input::CANCEL.to_string(),
                "Resets the desired pose.".to_string(),
            ],
            [
                input::CONFIRM.to_string(),
                "Sends the desired pose.".to_string(),
            ],
            [
                input::INCREMENT_STEP.to_string(),
                "Increases the step size for manipulating the desired pose.".to_string(),
            ],
            [
                input::DECREMENT_STEP.to_string(),
                "Decreases the step size for manipulating the desired pose.".to_string(),
            ],
            [
                input::NEXT.to_string(),
                "Switches to the next topic to which the poses are sent.".to_string(),
            ],
            [
                input::PREVIOUS.to_string(),
                "Switches to the previous topic to which the poses are sent.".to_string(),
            ],
        ];
        keymap.extend(self.viewport.borrow().get_keymap());
        keymap
    }
}

impl UseViewport for SendPose {
    fn draw_in_viewport(&self, ctx: &mut Context) {
        self.viewport.borrow().draw_in_viewport(ctx);
        if self.new_pose.abs_diff_ne(&self.robot_pose, 0.01) {
            let pose_estimate_ros = transformation::iso2d_to_ros(&self.new_pose);
            for elem in
                &get_current_footprint(&pose_estimate_ros, &self.viewport.borrow().footprint)
            {
                ctx.draw(&Line {
                    x1: elem.0,
                    y1: elem.1,
                    x2: elem.2,
                    y2: elem.3,
                    color: Color::Gray,
                });
            }
            for mut line in
                Viewport::get_frame_lines(&pose_estimate_ros, self.viewport.borrow().axis_length)
            {
                line.color = Color::Gray;
                ctx.draw(&line);
            }
        }
    }
    fn x_bounds(&self) -> [f64; 2] {
        let scale_factor = self.viewport.borrow().terminal_size.0 as f64
            / self.viewport.borrow().terminal_size.1 as f64
            * 0.5;
        [
            self.new_pose.translation.x
                + self.viewport.borrow().initial_bounds[0] / self.viewport.borrow().zoom
                    * scale_factor,
            self.new_pose.translation.x
                + self.viewport.borrow().initial_bounds[1] / self.viewport.borrow().zoom
                    * scale_factor,
        ]
    }
    fn y_bounds(&self) -> [f64; 2] {
        [
            self.new_pose.translation.y
                + self.viewport.borrow().initial_bounds[2] / self.viewport.borrow().zoom,
            self.new_pose.translation.y
                + self.viewport.borrow().initial_bounds[3] / self.viewport.borrow().zoom,
        ]
    }

    fn info(&self) -> String {
        format!(
            "Topic: /{}, Cursor step: {:.2}",
            &self.publishers[self.current_topic].get_topic(),
            &self.increment
        )
    }
}
