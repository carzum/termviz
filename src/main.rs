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
mod pose;
mod transformation;
use futures::{future::FutureExt, select, StreamExt};
use futures_timer::Delay;
use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Duration;

use clap::{Arg, Command};
use crossterm::{
    event::{DisableMouseCapture, Event, EventStream, KeyCode, KeyEvent, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, LeaveAlternateScreen},
};
use rosrust;
use rustros_tf::TfListener;
use std::error::Error;

#[tokio::main]
async fn main() -> Result<(), Box<dyn Error>> {
    // Terminal initialization

    let matches = Command::new("termviz")
        .about("ROS visualization on the terminal")
        .arg(
            Arg::new("config").long_help("Optional YAML file with a custom termviz configuration."),
        )
        .after_help("More documentation can be found at: https://github.com/carzum/termviz")
        .get_matches();

    let conf = config::get_config(matches.get_one("config"))?;

    println!("Connecting to ros...");
    rosrust::init("termviz");

    let mut key_to_input: HashMap<KeyCode, String> = conf
        .key_mapping
        .iter()
        .map(|(v, k)| match k.as_str() {
            "Enter" => (KeyCode::Enter, v.clone()),
            "Esc" => (KeyCode::Esc, v.clone()),
            _ => (KeyCode::Char(k.chars().next().unwrap()), v.clone()),
        })
        .collect();
    for i in 0..9 {
        key_to_input.insert(
            KeyCode::Char(std::char::from_digit(i, 10).unwrap()),
            i.to_string(),
        );
    }

    // Initialize listener and wait for it to come up
    println!(
        "Waiting for tf from {:?} to {:?} to become available...",
        conf.fixed_frame, conf.robot_frame
    );
    let listener = Arc::new(TfListener::new());
    while rosrust::is_ok() {
        let res =
            listener.lookup_transform(&conf.fixed_frame, &conf.robot_frame, rosrust::Time::new());
        match res {
            Ok(_res) => break,
            Err(_e) => {
                std::thread::sleep(std::time::Duration::from_millis(100));
                continue;
            }
        };
    }

    println!("Initiating terminal");

    let rate = Duration::from_millis(1000 / conf.target_framerate as u64);

    let default_app_config = Arc::new(Mutex::new(app::App::new(listener.clone(), conf)));

    let mut running_app = default_app_config.lock().unwrap();

    let mut terminal = running_app.init_terminal().unwrap();

    let mut reader = EventStream::new();
    loop {
        let mut event = reader.next().fuse();
        let mut delay = Delay::new(rate).fuse();

        select! {
            _ = delay => {
                running_app.run();
            },
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
