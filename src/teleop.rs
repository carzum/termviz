use crate::config::TeleopConfig;
use rosrust;
use rosrust_msg;
use std::collections::HashMap;
use termion::event::Key;

pub struct Teleoperator {
    current_velocities: Velocities,
    _cmd_val_pub: rosrust::Publisher<rosrust_msg::geometry_msgs::Twist>,
    _keys_to_direction: HashMap<Key, (String, i8)>,
    _increment: f64,
}

pub struct Velocities {
    x: f64,
    y: f64,
    theta: f64,
}

impl Teleoperator {
    pub fn new(config: TeleopConfig) -> Teleoperator {
        let cmd_val_publisher = rosrust::publish(&config.cmd_vel_topic, 1).unwrap();
        let initial_velocities = Velocities {
            x: 0.,
            y: 0.,
            theta: 0.,
        };
        Teleoperator {
            _cmd_val_pub: cmd_val_publisher,
            current_velocities: initial_velocities,
            // Map strings to Key::Char
            _keys_to_direction: config
                .key_mapping
                .iter()
                .map(|(k, v)| (Key::Char(k.chars().next().unwrap()), v.clone()))
                .collect(),
            _increment: config.increment.clone(),
        }
    }

    pub fn handle_input_from_key(&mut self, input: Key) {
        if !self._keys_to_direction.contains_key(&input) {
            self.reset();
            return;
        }
        let (coordinate, direction) = self._keys_to_direction[&input].clone();
        match coordinate.as_ref() {
            "x" => self.current_velocities.x += direction as f64 * self._increment,
            "y" => self.current_velocities.y += direction as f64 * self._increment,
            "theta" => self.current_velocities.theta += direction as f64 * self._increment,
            _ => println!("{} is not supported!", coordinate),
        }
    }

    pub fn run(&mut self) {
        let mut vel_cmd = rosrust_msg::geometry_msgs::Twist::default();
        vel_cmd.linear.x = self.current_velocities.x;
        vel_cmd.linear.y = self.current_velocities.y;
        vel_cmd.angular.z = self.current_velocities.theta;
        self._cmd_val_pub.send(vel_cmd).unwrap();
    }

    pub fn reset(&mut self) {
        self.current_velocities = Velocities {
            x: 0.,
            y: 0.,
            theta: 0.,
        };
    }
}
