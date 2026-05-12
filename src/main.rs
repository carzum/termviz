mod app;
mod app_modes;
mod config;
mod footprint;
mod image;
mod laser;
mod listeners;
mod map;
mod marker;
mod pointcloud;
mod polygon;
mod pose;
mod ros;
mod transformation;
use futures::{future::FutureExt, select, StreamExt};
use futures_timer::Delay;
use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use clap::{value_parser, Arg, ArgAction, Command};
use colored::Colorize;
use crossterm::{
    event::{DisableMouseCapture, Event, EventStream, KeyCode, KeyEvent, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, LeaveAlternateScreen},
};
use dialoguer::Confirm;
use std::error::Error;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // Terminal initialization

    let matches = Command::new("termviz")
        .about("ROS visualization on the terminal")
        .arg(
            Arg::new("config")
                .long_help("Optional YAML file with a custom termviz configuration.")
                .value_parser(value_parser!(PathBuf)),
        )
        .arg(
            Arg::new("tf-wait-time")
                .long("tf-wait-time")
                .short('t')
                .action(ArgAction::Set)
                .default_value("1")
                .long_help("How long to wait for the robot pose TF on startup, in seconds.")
                .value_parser(value_parser!(u64)),
        )
        .after_help("More documentation can be found at: https://github.com/carzum/termviz")
        .get_matches();

    let conf = config::get_config(matches.get_one("config"))?;

    println!("Connecting to ROS...");
    ros::init("termviz")?;

    let mut key_to_input: HashMap<KeyCode, String> = conf
        .key_mapping
        .iter()
        .map(|(v, k)| match k.as_str() {
            "Enter" => (KeyCode::Enter, v.clone()),
            "Esc" => (KeyCode::Esc, v.clone()),
            "Space" => (KeyCode::Char(' '), v.clone()),
            _ => (KeyCode::Char(k.chars().next().unwrap()), v.clone()),
        })
        .collect();
    for i in 0..9 {
        key_to_input.insert(
            KeyCode::Char(std::char::from_digit(i, 10).unwrap()),
            i.to_string(),
        );
    }

    let tf = ros::tf_client();

    // rustros_tf has no option for a timeout, so we have to do it manually.
    let mut passed_time = std::time::Duration::ZERO;
    let max_time = std::time::Duration::from_secs(*matches.get_one::<u64>("tf-wait-time").unwrap());
    let sleep_time = std::time::Duration::from_millis(100);

    println!("Waiting up to {}s for robot pose...", max_time.as_secs());
    let robot_pose_available = loop {
        if tf
            .lookup_transform(&conf.fixed_frame, &conf.robot_frame, ros::now())
            .is_ok()
        {
            break true;
        }
        std::thread::sleep(sleep_time);
        passed_time += sleep_time;
        if passed_time > max_time {
            break false;
        }
    };

    if !robot_pose_available {
        println!(
            "\n{}\n{}",
            "Robot pose is not being published on TF!".bold().red(),
            "termviz will display the robot at the origin of the map and you can set the pose from there."
        );
        if !Confirm::new()
            .with_prompt("\nContinue?")
            .interact()
            .unwrap()
        {
            Err("Aborting.")?;
        }
    }

    println!("Initiating terminal");

    let rate = Duration::from_millis(1000 / conf.target_framerate as u64);

    let default_app_config = Arc::new(Mutex::new(app::App::new(tf.clone(), conf)));

    let mut running_app = default_app_config.lock().unwrap();

    let mut terminal = running_app.init_terminal().unwrap();

    let mut reader = EventStream::new();
    loop {
        let mut event = reader.next().fuse();
        let mut delay = Delay::new(rate).fuse();

        select! {
            _ = delay => (),
            maybe_event = event => {
                match maybe_event {
                    Some(Ok(event)) => {
                        if event == Event::Key(KeyEvent{code:KeyCode::Char('c'), modifiers: KeyModifiers::CONTROL}) {
                            break;
                        }
                        if let Event::Key(input) = event {

                            if key_to_input.contains_key(&input.code) {
                                running_app.handle_input(&key_to_input[&input.code]);
                            } else {
                                running_app.handle_input(&app_modes::input::UNMAPPED.to_string());
                            }
                        }

                    }
                    Some(Err(e)) => println!("Error: {:?}\r", e),
                    None => break,
                }
            }
        };
        running_app.run();
        terminal.draw(|f| {
            running_app.draw(f);
        })?;
    }
    // restore terminal
    disable_raw_mode()?;
    execute!(
        terminal.backend_mut(),
        LeaveAlternateScreen,
        DisableMouseCapture
    )?;
    terminal.show_cursor()?;
    Ok(())
}
