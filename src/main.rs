mod app;
mod app_modes;
mod config;
mod event;
mod footprint;
mod image;
mod laser;
mod listeners;
mod map;
mod marker;
mod pointcloud;
mod pose;
mod transformation;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use clap::{Arg, Command};
use event::{Config, Event, Events};
use rosrust;
use rustros_tf;
use std::error::Error;
use termion::event::Key;

fn main() -> Result<(), Box<dyn Error>> {
    // Terminal initialization

    let matches = Command::new("termviz")
        .about("ROS visualization on the terminal")
        .arg(
            Arg::new("config").long_help("Optional YAML file with a custom termviz configuration."),
        )
        .after_help("More documentation can be found at: https://github.com/carzum/termviz")
        .get_matches();

    let conf = config::get_config(matches.get_one("config")).unwrap();
    println!("Connecting to ros...");
    rosrust::init("termviz");

    let static_frame = conf.fixed_frame.clone();
    let mut key_to_input: HashMap<Key, String> = conf
        .key_mapping
        .iter()
        .map(|(v, k)| match k.as_str() {
            "Enter" => (Key::Char('\n'), v.clone()),
            "Esc" => (Key::Esc, v.clone()),
            _ => (Key::Char(k.chars().next().unwrap()), v.clone()),
        })
        .collect();
    for i in 0..9 {
        key_to_input.insert(
            Key::Char(std::char::from_digit(i, 10).unwrap()),
            i.to_string(),
        );
    }
    // Initialize listener and wait for it to come up
    println!("Waiting for tf...");
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

    println!("Initiating terminal");

    let config = Config {
        tick_rate: Duration::from_millis(1000 / conf.target_framerate as u64),
        ..Default::default()
    };
    let events = Events::with_config(config);

    let default_app_config = Arc::new(Mutex::new(app::App::new(listener.clone(), conf)));

    let mut running_app = default_app_config.lock().unwrap();
    let mut terminal = running_app.init_terminal().unwrap();
    loop {
        running_app.run();
        terminal.draw(|f| {
            running_app.draw(f);
        })?;
        match events.next()? {
            Event::Input(input) => match input {
                Key::Ctrl('c') => break,
                _ => {
                    if key_to_input.contains_key(&input) {
                        running_app.handle_input(&key_to_input[&input]);
                    } else {
                        running_app.handle_input(&app_modes::input::UNMAPPED.to_string());
                    }
                }
            },
            Event::Tick => {}
        }
    }
    Ok(())
}
