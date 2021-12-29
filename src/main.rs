mod config;
mod event;
mod laser;
mod footprint;
mod map;
mod app;
mod initial_pose;
mod transformation;
mod marker;
mod listeners;
use std::time::Duration;
use std::sync::{Arc, Mutex, RwLock};
use std::thread;

use termion::event::Key;
use std::error::Error;
use rosrust;
use rosrust_msg;
use rustros_tf;
use event::{Config, Event, Events};


fn main() -> Result<(), Box<dyn Error>> {
    // Terminal initialization
    let conf = config::get_config().unwrap();
    println!("Connecting to ros...");
    rosrust::init("termviz");
    println!("Retrieving map...");

    let static_frame  = conf.fixed_frame;

    let current_tf = Arc::new(RwLock::new(
            rosrust_msg::geometry_msgs::Transform::default()));
    let cb_tf = current_tf.clone();

    println!("spawning tf listener");
    let listener = Arc::new(Mutex::new(rustros_tf::TfListener::new()));
    let tf_listener = listener.clone();
    let sleep = 1000 / conf.target_framerate as u64;
    let _static_frame = static_frame.clone();
    let _tf_handle = thread::spawn(move|| {
        // update robot position thread
        while rosrust::is_ok() {
            {
            let tf_listener = tf_listener.lock().unwrap();
            let res = &tf_listener.lookup_transform(&_static_frame,
                                                    "base_link",
                                                    rosrust::Time::new());
            if res.is_ok() {
                let mut cb_tf = cb_tf.write().unwrap();
                *cb_tf = res.as_ref().unwrap().transform.clone();
                }
            }
            thread::sleep(Duration::from_millis(sleep));
        }
    });

    let initial_pose_pub = initial_pose::InitialPosePub::new(
        "initial_pose",
        current_tf.clone(),
        static_frame.clone());

    println!("Initiating terminal");

    let config = Config {
        tick_rate: Duration::from_millis(sleep),
        ..Default::default()
    };
    let events = Events::with_config(config);

    let mut distance = 0.1;
    let default_app_config = Arc::new(Mutex::new(app::App::default()));
    let mut running_app = default_app_config.lock().unwrap();
    let mut terminal = running_app.init_terminal().unwrap();


    loop {
        match running_app.mode {
            app::AppModes::RobotView => {
                terminal.draw( |f| {
                    running_app.compute_bounds(&current_tf);
                    running_app.draw_robot(f, &current_tf);
                })?;
                match events.next()? {
                    Event::Input(input) => match input {
                        Key::Char('q') => {
                            initial_pose_pub.send_estimate(0.0, 0.0, distance);
                        }
                        Key::Char('e') => {
                            initial_pose_pub.send_estimate(0.0, 0.0, - distance);
                        }
                        Key::Char('w') => {
                            initial_pose_pub.send_estimate(distance, 0.0, 0.0);
                        }
                        Key::Char('s') => {
                            initial_pose_pub.send_estimate(- distance, 0.0, 0.0);
                        }
                        Key::Char('d') => {
                            initial_pose_pub.send_estimate(0.0, distance, 0.0);
                        }
                        Key::Char('a') => {
                            initial_pose_pub.send_estimate(0.0, - distance, 0.0);
                        }
                        Key::Char('-') => {
                            running_app.decrease_zoom();
                            running_app.compute_bounds(&current_tf);
                        }
                        Key::Char('=') => {
                            running_app.increase_zoom();
                            running_app.compute_bounds(&current_tf);
                        }
                        Key::Ctrl('c') => {
                            break;
                        }
                        Key::Char('k') => {
                            distance = distance * 2.;
                        }
                        Key::Char('j') => {
                            distance = distance * 0.5;
                        }
                        Key::Char('h') => {
                            running_app.mode = app::AppModes::HelpPage;
                        }
                        _ => {}
                    },
                    Event::Tick => {}
                }
            }
            app::AppModes::HelpPage => {
                terminal.draw(|f| {
                    running_app.show_help(f);
                })?;
                match events.next()? {
                    Event::Input(input) => match input {
                        Key::Ctrl('c') => {
                            break;
                        }
                        _ => {
                            running_app.mode = app::AppModes::RobotView;
                        }
                    },
                    Event::Tick => {}
                }
            }
        }
    }
    Ok(())
}
