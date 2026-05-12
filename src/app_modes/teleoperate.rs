use crate::app_modes::viewport::{UseViewport, Viewport};
use crate::app_modes::{input, AppMode, BaseMode};
use crate::config::{TeleopConfig, TeleopMode};
use crate::ros;
use crate::ros::types;
use std::cell::RefCell;
use std::rc::Rc;
use tui::backend::Backend;
use tui::widgets::canvas::Context;

pub struct Teleoperate {
    viewport: Rc<RefCell<Viewport>>,
    current_velocities: Velocities,
    cmd_vel_pub: ros::TwistPublisher,
    increment: f64,
    increment_step: f64,
    publish_cmd_vel_when_idle: bool,
    has_published_zero_once: bool,
    mode: TeleopMode,
    max_vel: f64,
}

pub struct Velocities {
    x: f64,
    y: f64,
    theta: f64,
}

fn move_towards_zero(mut num: f64, increment: f64) -> f64 {
    if num > 0.0 {
        num = (num - increment).max(0.0);
    } else {
        num = (num + increment).min(0.0);
    }
    num
}

impl Teleoperate {
    pub fn new(viewport: Rc<RefCell<Viewport>>, config: TeleopConfig) -> Teleoperate {
        let cmd_vel_publisher = ros::publish_twist(&config.cmd_vel_topic, 1).unwrap();
        let initial_velocities = Velocities {
            x: 0.,
            y: 0.,
            theta: 0.,
        };
        Teleoperate {
            viewport: viewport,
            cmd_vel_pub: cmd_vel_publisher,
            current_velocities: initial_velocities,
            increment: config.default_increment,
            increment_step: config.increment_step,
            publish_cmd_vel_when_idle: config.publish_cmd_vel_when_idle,
            has_published_zero_once: true, // Initialize to true so the robot is not stopped when entering the mode
            mode: config.mode,
            max_vel: config.max_vel,
        }
    }
}

impl<B: Backend> BaseMode<B> for Teleoperate {}

impl Teleoperate {
    fn publish_current_cmd_val(&mut self) {
        ros::send_twist(
            &self.cmd_vel_pub,
            types::Twist {
                linear_x: self.current_velocities.x,
                linear_y: self.current_velocities.y,
                angular_z: self.current_velocities.theta,
            },
        );
    }
}

impl AppMode for Teleoperate {
    fn handle_input(&mut self, input: &String) {
        self.viewport.borrow_mut().handle_input(input);
        if self.mode == TeleopMode::Classic {
            match input.as_str() {
                input::UP => {
                    self.current_velocities.x =
                        (self.current_velocities.x + 1 as f64 * self.increment).min(self.max_vel)
                }
                input::DOWN => {
                    self.current_velocities.x =
                        (self.current_velocities.x - 1 as f64 * self.increment).min(-self.max_vel)
                }
                input::LEFT => {
                    self.current_velocities.y =
                        (self.current_velocities.y + 1 as f64 * self.increment).min(self.max_vel)
                }
                input::RIGHT => {
                    self.current_velocities.y =
                        (self.current_velocities.y - 1 as f64 * self.increment).min(-self.max_vel)
                }
                input::ROTATE_LEFT => {
                    self.current_velocities.theta = (self.current_velocities.theta
                        + 1 as f64 * self.increment)
                        .min(self.max_vel)
                }
                input::ROTATE_RIGHT => {
                    self.current_velocities.theta = (self.current_velocities.theta
                        - 1 as f64 * self.increment)
                        .min(-self.max_vel)
                }
                input::INCREMENT_STEP => self.increment += self.increment_step,
                input::DECREMENT_STEP => {
                    self.increment = self
                        .increment_step
                        .max(self.increment - self.increment_step)
                }
                _ => self.reset(),
            }
        } else {
            match input.as_str() {
                input::UP => {
                    let new_vel =
                        (self.current_velocities.x + 2 as f64 * self.increment).min(self.max_vel);
                    self.current_velocities.x = new_vel;
                }
                input::DOWN => {
                    let new_vel =
                        (self.current_velocities.x - 2 as f64 * self.increment).max(-self.max_vel);
                    self.current_velocities.x = new_vel;
                }
                input::LEFT => {
                    let new_vel =
                        (self.current_velocities.y + 2 as f64 * self.increment).min(self.max_vel);
                    self.current_velocities.y = new_vel;
                }
                input::RIGHT => {
                    let new_vel =
                        (self.current_velocities.y - 2 as f64 * self.increment).max(-self.max_vel);
                    self.current_velocities.y = new_vel;
                }
                input::ROTATE_LEFT => {
                    let new_vel = (self.current_velocities.theta + 2 as f64 * self.increment)
                        .min(self.max_vel);
                    self.current_velocities.theta = new_vel;
                }
                input::ROTATE_RIGHT => {
                    let new_vel = (self.current_velocities.theta - 2 as f64 * self.increment)
                        .max(-self.max_vel);
                    self.current_velocities.theta = new_vel;
                }
                input::INCREMENT_STEP => self.increment += self.increment_step,
                input::DECREMENT_STEP => {
                    self.increment = self
                        .increment_step
                        .max(self.increment - self.increment_step)
                }
                _ => self.reset(),
            }
        }
    }

    fn run(&mut self) {
        // If the velocity is reset to 0 only publish it once
        // this prevents the robot from being blocked if the
        // app mode is not closed
        if self.mode == TeleopMode::Safe {
            self.current_velocities.x =
                move_towards_zero(self.current_velocities.x, self.increment);
            self.current_velocities.y =
                move_towards_zero(self.current_velocities.y, self.increment);
            self.current_velocities.theta =
                move_towards_zero(self.current_velocities.theta, self.increment);
        };
        if !self.publish_cmd_vel_when_idle
            && self.current_velocities.x == 0 as f64
            && self.current_velocities.y == 0 as f64
            && self.current_velocities.theta == 0 as f64
        {
            // If we did not publish the stop, do it once
            if !self.has_published_zero_once {
                self.has_published_zero_once = true;
                self.publish_current_cmd_val()
            }
        } else {
            // Otherwise just publish
            self.has_published_zero_once = false;
            self.publish_current_cmd_val()
        };
    }

    fn reset(&mut self) {
        self.current_velocities = Velocities {
            x: 0.,
            y: 0.,
            theta: 0.,
        };
        self.run(); // Send 0 velocities just in case
    }

    fn get_name(&self) -> String {
        "Teleoperate".to_string()
    }

    fn get_description(&self) -> Vec<String> {
        vec![
            "This mode allows to teleoperate the robot by publishing velocity commands on the given topic.".to_string(),
            "The viewport is centered on the robot.".to_string(),
            "Two modes are possible: Classic and Safe, check the config for more details".to_string(),
        ]
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        let mut keymap = vec![
            [
                input::UP.to_string(),
                "Moves positively along the x axis.".to_string(),
            ],
            [
                input::DOWN.to_string(),
                "Moves negatively along the x axis.".to_string(),
            ],
            [
                input::RIGHT.to_string(),
                "Moves positively along the y axis.".to_string(),
            ],
            [
                input::LEFT.to_string(),
                "Moves negatively along the y axis.".to_string(),
            ],
            [
                input::ROTATE_LEFT.to_string(),
                "Rotates counter-clockwise.".to_string(),
            ],
            [
                input::ROTATE_RIGHT.to_string(),
                "Rotates clockwise.".to_string(),
            ],
            [
                input::INCREMENT_STEP.to_string(),
                "Increases the velocity step.".to_string(),
            ],
            [
                input::DECREMENT_STEP.to_string(),
                "Decreases the velocity step.".to_string(),
            ],
        ];
        keymap.extend(self.viewport.borrow().get_keymap());
        keymap.push([
            input::UNMAPPED.to_string(),
            "Reset the velocities.".to_string(),
        ]);
        keymap
    }
}

impl UseViewport for Teleoperate {
    fn draw_in_viewport(&self, ctx: &mut Context) {
        self.viewport.borrow().draw_in_viewport(ctx);
    }

    fn x_bounds(&self) -> [f64; 2] {
        self.viewport.borrow().x_bounds()
    }

    fn y_bounds(&self) -> [f64; 2] {
        self.viewport.borrow().y_bounds()
    }

    fn info(&self) -> String {
        format!(
            "Velocity step: {:.2}, Mode: {}, Velocities: x: {:.2}, y: {:.2}, theta: {:.2}",
            &self.increment,
            self.mode,
            self.current_velocities.x,
            self.current_velocities.y,
            self.current_velocities.theta,
        )
    }
}
