mod app;
mod config;
mod event;
mod footprint;
mod initial_pose;
mod laser;
mod listeners;
mod map;
mod marker;
mod transformation;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use event::{Config, Event, Events};
use rosrust;
use rustros_tf;
use std::error::Error;
use termion::event::Key;

fn main() -> Result<(), Box<dyn Error>> {
    // Terminal initialization
    let conf = config::get_config().unwrap();
    println!("Connecting to ros...");
    rosrust::init("termviz");
    println!("Retrieving map...");

    let static_frame = conf.fixed_frame;

    // Initialize listener and wait for it to come up
    let listener = Arc::new(rustros_tf::TfListener::new());
    while rosrust::is_ok() {
        let res =
            listener.lookup_transform("base_link", &static_frame.clone(), rosrust::Time::new());
        match res {
            Ok(_res) => break,
            Err(_e) => continue,
        };
    }

    let initial_pose_pub =
        initial_pose::InitialPosePub::new("base_link", static_frame.clone(), listener.clone());

    println!("Initiating terminal");

    let config = Config {
        tick_rate: Duration::from_millis(1000 / conf.target_framerate as u64),
        ..Default::default()
    };
    let events = Events::with_config(config);

    let mut distance = 0.1;
    let default_app_config = Arc::new(Mutex::new(app::App::new(listener.clone())));

    let mut running_app = default_app_config.lock().unwrap();
    let mut terminal = running_app.init_terminal().unwrap();

    loop {
        match running_app.mode {
            app::AppModes::RobotView => {
                terminal.draw(|f| {
                    running_app.compute_bounds(listener.clone());
                    running_app.draw_robot(f, listener.clone());
                })?;
                match events.next()? {
                    Event::Input(input) => match input {
                        Key::Char('q') => {
                            initial_pose_pub.send_estimate(0.0, 0.0, distance);
                        }
                        Key::Char('e') => {
                            initial_pose_pub.send_estimate(0.0, 0.0, -distance);
                        }
                        Key::Char('w') => {
                            initial_pose_pub.send_estimate(distance, 0.0, 0.0);
                        }
                        Key::Char('s') => {
                            initial_pose_pub.send_estimate(-distance, 0.0, 0.0);
                        }
                        Key::Char('d') => {
                            initial_pose_pub.send_estimate(0.0, distance, 0.0);
                        }
                        Key::Char('a') => {
                            initial_pose_pub.send_estimate(0.0, -distance, 0.0);
                        }
                        Key::Char('-') => {
                            running_app.decrease_zoom();
                            running_app.compute_bounds(listener.clone());
                        }
                        Key::Char('=') => {
                            running_app.increase_zoom();
                            running_app.compute_bounds(listener.clone());
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
