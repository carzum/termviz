mod app;
mod config;
mod event;
mod footprint;
mod initial_pose;
mod laser;
mod listeners;
mod map;
mod marker;
mod rosout;
mod teleop;
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
        let res = listener.lookup_transform(
            &conf.robot_frame,
            &static_frame.clone(),
            rosrust::Time::new(),
        );
        match res {
            Ok(_res) => break,
            Err(_e) => continue,
        };
    }

    let initial_pose_pub =
        initial_pose::InitialPosePub::new(&conf.robot_frame, static_frame.clone());

    println!("Initiating terminal");

    let config = Config {
        tick_rate: Duration::from_millis(1000 / conf.target_framerate as u64),
        ..Default::default()
    };
    let mut teleoperator = teleop::Teleoperator::new(conf.teleop_key_mapping);
    let events = Events::with_config(config);

    let mut distance = 0.1;
    let default_app_config = Arc::new(Mutex::new(app::App::new(listener.clone())));

    let mut running_app = default_app_config.lock().unwrap();
    let mut terminal = running_app.init_terminal().unwrap();

    loop {
        match running_app.mode {
            app::AppModes::RobotView | app::AppModes::SendPose | app::AppModes::Teleoperate => {
                terminal.draw(|f| {
                    running_app.compute_bounds(listener.clone());
                    running_app.draw_robot(f, listener.clone());
                    match running_app.mode {
                        app::AppModes::Teleoperate => {
                            teleoperator.run();
                        }
                        _ => (),
                    }
                })?;
                match events.next()? {
                    Event::Input(input) => {
                        if running_app.mode == app::AppModes::Teleoperate {
                            teleoperator.handle_input_from_key(input);
                        }
                        match input {
                            Key::Char('q') => {
                                if running_app.mode != app::AppModes::Teleoperate {
                                    running_app.mode = app::AppModes::SendPose;
                                    running_app.move_pose_estimate(0.0, 0.0, distance);
                                }
                            }
                            Key::Char('e') => {
                                if running_app.mode != app::AppModes::Teleoperate {
                                    running_app.mode = app::AppModes::SendPose;
                                    running_app.move_pose_estimate(0.0, 0.0, -distance);
                                }
                            }
                            Key::Char('w') => {
                                if running_app.mode != app::AppModes::Teleoperate {
                                    running_app.mode = app::AppModes::SendPose;
                                    running_app.move_pose_estimate(distance, 0.0, 0.0);
                                }
                            }
                            Key::Char('s') => {
                                if running_app.mode != app::AppModes::Teleoperate {
                                    running_app.mode = app::AppModes::SendPose;
                                    running_app.move_pose_estimate(-distance, 0.0, 0.0);
                                }
                            }
                            Key::Char('d') => {
                                if running_app.mode != app::AppModes::Teleoperate {
                                    running_app.mode = app::AppModes::SendPose;
                                    running_app.move_pose_estimate(0.0, -distance, 0.0);
                                }
                            }
                            Key::Char('a') => {
                                if running_app.mode != app::AppModes::Teleoperate {
                                    running_app.mode = app::AppModes::SendPose;
                                    running_app.move_pose_estimate(0.0, distance, 0.0);
                                }
                            }
                            Key::Esc => {
                                running_app.mode = app::AppModes::RobotView;
                                teleoperator.reset();
                            }
                            Key::Char('\n') => {
                                if running_app.mode != app::AppModes::Teleoperate {
                                    initial_pose_pub
                                        .send_estimate(&running_app.get_pose_estimate());
                                    running_app.mode = app::AppModes::RobotView;
                                }
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
                            Key::Char('t') => match running_app.mode {
                                app::AppModes::Teleoperate => {
                                    teleoperator.reset();
                                    teleoperator.run();
                                    running_app.mode = app::AppModes::RobotView;
                                }
                                _ => running_app.mode = app::AppModes::Teleoperate,
                            },
                            Key::Char('L') => {
                                running_app.toggle_rosout_widget();
                            }
                            Key::Char('h') => {
                                running_app.mode = app::AppModes::HelpPage;
                            }
                            _ => {}
                        }
                    }
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
