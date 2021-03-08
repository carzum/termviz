mod config;
mod event;
mod laser;
mod footprint;
mod initial_pose;
mod transformation;
mod marker;
mod listeners;
use std::io;
use std::time::Duration;
use std::sync::{Arc, Mutex, RwLock, Condvar};
use std::thread;

use termion::event::Key;
use termion::input::MouseTerminal;
use termion::raw::IntoRawMode;
use termion::screen::AlternateScreen;
use termion::terminal_size;
use tui::backend::TermionBackend;
use tui::layout::{Constraint, Layout};
use tui::style::Color;
use tui::widgets::canvas::{Canvas, Line, Points};
use tui::widgets::{Block, Borders};
use tui::Terminal;

use std::error::Error;
use rosrust;
use rustros_tf;
use nalgebra::geometry::{Quaternion, UnitQuaternion, Point3, Isometry3, Translation3};
use event::{Config, Event, Events};


pub fn compute_bounds(tf: &std::sync::RwLockReadGuard<rustros_tf::msg::geometry_msgs::Transform>,
                      zoom: f64, scale_factor: f64) -> Vec<f64> {
        vec![tf.translation.x - 5.0 / zoom * scale_factor,
        tf.translation.x + 5.0 / zoom * scale_factor,
        tf.translation.y - 5.0 / zoom,
        tf.translation.y + 5.0 / zoom]
}

pub fn retrieve_map(topic: &str) -> rosrust_msg::nav_msgs::OccupancyGrid {

    let pair = Arc::new((Mutex::new(false), Condvar::new()));
    let pair2 = pair.clone();
    let map = Arc::new(Mutex::new(rosrust_msg::nav_msgs::OccupancyGrid::default()));
    let cb_map = map.clone();

    let _map_sub = rosrust::subscribe(topic, 1, move |v: rosrust_msg::nav_msgs::OccupancyGrid| {
        // Callback for handling received messages
        // let mut map = map.lock().unwrap();
        // Arc::get_mut_unchecked(&mut map).as_mut_ptr().write(v);
        let (lock, cvar) = &*pair2;
        let mut started = lock.lock().unwrap();
        let mut cb_map = cb_map.lock().unwrap();
        *cb_map = v;
        *started = true;
        cvar.notify_one();
    })
    .unwrap();

    let (lock, cvar) = &*pair;
    let mut started = lock.lock().unwrap();
    while !*started {
        started = cvar.wait(started).unwrap();
    }
    // map.try_unwrap().unwrap().into_inner().unwrap()
    return (*map.lock().unwrap()).clone();
}


pub fn get_map_points(map: &rosrust_msg::nav_msgs::OccupancyGrid) -> Vec<(f64, f64)>  {
    let mut occ_points: Vec<(f64, f64)> = Vec::new();
    let tra = Translation3::new(map.info.origin.position.x,
                                map.info.origin.position.y,
                                map.info.origin.position.z);
    let rot = UnitQuaternion::new_normalize(Quaternion::new(
            map.info.origin.orientation.w,
            map.info.origin.orientation.x,
            map.info.origin.orientation.y,
            map.info.origin.orientation.z,
            ));
    let isometry = Isometry3::from_parts(tra, rot);
    for (i, pt) in map.data.iter().enumerate() {
        let line = i / map.info.width as usize;
        let column = i - line * map.info.width as usize;
        if pt != &0 {
            let transformed_point = isometry.transform_point(&Point3::new(
                    (column as f64) * map.info.resolution as f64,
                    line as f64 * map.info.resolution as f64,
                    0.));
            if pt > &0 {
                occ_points.push((transformed_point.x, transformed_point.y));
            }
            // else {
            //     unknown_points.push((x, y));
            // }
        }
    };
    occ_points
}


pub fn get_frame_lines(tf: &std::sync::RwLockReadGuard<rustros_tf::msg::geometry_msgs::Transform>) -> Vec<Line> {
        let mut result: Vec<Line> = Vec::new();
        let base_x = transformation::transform_relative_pt(&tf, (0.1, 0.0));
        let base_y = transformation::transform_relative_pt(&tf, (0.0, 0.1));
        result.push(Line {
            x1: tf.translation.x,
            y1: tf.translation.y,
            x2: base_x.0,
            y2: base_x.1,
            color: Color::Red,
        });
        result.push(Line {
            x1: tf.translation.x,
            y1: tf.translation.y,
            x2: base_y.0,
            y2: base_y.1,
            color: Color::Green,
        });
        result
}


fn main() -> Result<(), Box<dyn Error>> {
    // Terminal initialization
    let conf = config::get_config().unwrap();
    println!("Connecting to ros...");
    rosrust::init("termviz");
    println!("Retrieving map...");
    let map = retrieve_map(&conf.map_topics[0]);
    println!("got map! size {} {}", map.info.width, map.info.height);
    let occ_points = get_map_points(&map);
    // println!("Retrieving markers...");
    // println!("got markers...");

    let static_frame  = map.header.frame_id.clone();
    let tf = Arc::new(RwLock::new(
            rustros_tf::msg::geometry_msgs::Transform::default()));
    let cb_tf = tf.clone();

    println!("spawning tf listener");
    let listener = Arc::new(Mutex::new(rustros_tf::TfListener::new()));
    let tf_listener = listener.clone();
    let sleep = 1000 / conf.target_framerate as u64;
    let _tf_handle = thread::spawn(move|| {
        // update robot position thread
        while rosrust::is_ok() {
            {
            let tf_listener = tf_listener.lock().unwrap();
            let res = &tf_listener.lookup_transform(&map.header.frame_id,
                                                    "base_link",
                                                    rosrust::Time::new());
            if res.is_ok() {
                let mut cb_tf = cb_tf.write().unwrap();
                *cb_tf = res.as_ref().unwrap().transform.clone();
            }}
            thread::sleep(Duration::from_millis(sleep));
        }
    });

    println!("getting laser scans");
    let listeners = listeners::Listeners::new(
        listener.clone(), static_frame.clone(),
        conf.laser_topics, conf.marker_array_topics);
    let initial_pose_pub = initial_pose::InitialPosePub::new(
        "initial_pose", listener.clone(), tf.clone(), static_frame.clone());

    let footprint_poly = footprint::get_footprint();


    // thread::sleep(Duration::from_secs(10));
    println!("Initiatiing terminal");

    let stdout = io::stdout().into_raw_mode()?;
    let stdout = MouseTerminal::from(stdout);
    let stdout = AlternateScreen::from(stdout);
    let backend = TermionBackend::new(stdout);
    let mut terminal = Terminal::new(backend)?;
    let tsize = terminal_size().unwrap();
    // get the right aspec ratio for braille rendering
    let size_factor = tsize.0 as f64 / tsize.1 as f64 * 0.5;

    let config = Config {
        tick_rate: Duration::from_millis(sleep),
        ..Default::default()
    };
    let events = Events::with_config(config);

    let mut zoom = 1.0;
    let mut distance = 0.1;

    loop {
        let tf_cp = tf.read().unwrap();
        let footprint_lines = footprint::get_current_footprint(
            &tf.read().unwrap(),&footprint_poly);
        let bounds = compute_bounds(&tf.read().unwrap(), zoom, size_factor);
        terminal.draw(|f| {
            let chunks = Layout::default()
                .constraints([Constraint::Percentage(100)].as_ref())
                .split(f.size());

            let canvas = Canvas::default()
                .block(
                    Block::default()
                        .title(format!("position: {} {} ", tf_cp.translation.x,
                                       tf_cp.translation.y))
                        .borders(Borders::NONE),
                )
                .x_bounds([bounds[0], bounds[1]])
                .y_bounds([bounds[2], bounds[3]])
                .paint(|ctx|{
                        ctx.draw(&Points {
                            coords: &occ_points,
                            color: Color::Rgb(200, 200, 200),
                            });
                        ctx.layer();
                        for elem in &footprint_lines {
                            ctx.draw(&Line {
                                x1: elem.0,
                                y1: elem.1,
                                x2: elem.2,
                                y2: elem.3,
                                color: Color::Blue,
                            });
                        };
                        for laser in &listeners.lasers {
                            ctx.draw(&Points {
                                coords: &laser.points.read().unwrap(),
                                color: Color::Red,
                            });
                        }
                        for marker in &listeners.markers {
                            for line in marker.get_lines() {
                                ctx.draw(&line);
                            };
                        }
                        for line in get_frame_lines(&tf.read().unwrap()) { ctx.draw(&line); };
                        let pos = &tf.as_ref().read().unwrap().translation;
                        ctx.print(pos.x, pos.y,"base_link", Color::White);
                });

            f.render_widget(canvas, chunks[0]);
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
                    zoom -= 0.1;
                }
                Key::Char('=') => {
                    zoom += 0.1;
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


                _ => {}
            },
            Event::Tick => {}
        }
    }

    Ok(())
}
