use crate::config::get_config;
use crate::listeners::Listeners;
use crate::footprint::{get_footprint, get_current_footprint};
use crate::transformation;
use rosrust_msg;
use rustros_tf::TfListener;
use tui::Frame;
use tui::widgets::canvas::{Canvas, Line, Points};
use tui::widgets::{Block, Borders};
use tui::backend::Backend;
use std::sync::{Arc, Mutex, RwLock};
use tui::style::Color;
use tui::layout::{Constraint, Layout};
use termion::terminal_size;
use termion::input::MouseTerminal;
use termion::raw::IntoRawMode;
use termion::raw::RawTerminal;
use std::io;
use termion::screen::AlternateScreen;
use tui::backend::TermionBackend;
use tui::Terminal;

pub enum AppModes{
    RobotView,
}

pub fn compute_bounds(
            ref_transform: &Arc<RwLock<rosrust_msg::geometry_msgs::Transform>>,
            zoom: f64,
            scale_factor: f64) -> Vec<f64> {
        let tf = &ref_transform.as_ref().read().unwrap();
        vec![tf.translation.x - 5.0 / zoom * scale_factor,
        tf.translation.x + 5.0 / zoom * scale_factor,
        tf.translation.y - 5.0 / zoom,
        tf.translation.y + 5.0 / zoom]
}

pub fn get_frame_lines(
            ref_transform: &Arc<RwLock<rosrust_msg::geometry_msgs::Transform>>, axis_length: f64
        ) -> Vec<Line> {
        let tf = &ref_transform.as_ref().read().unwrap();
        let mut result: Vec<Line> = Vec::new();
        let base_x = transformation::transform_relative_pt(&tf, (axis_length, 0.0));
        let base_y = transformation::transform_relative_pt(&tf, (0.0, axis_length));
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

pub struct App{
    pub mode: AppModes,
    pub tf_listener: Arc<Mutex<TfListener>>,
    pub listeners: Listeners,
    pub terminal_size: (u16, u16),
    pub bounds: Vec<f64>,
    pub zoom: f64,
    pub footprint_poly: Vec<(f64, f64)>,
    pub axis_length: f64,
}

impl Default for App {
    fn default() -> Self{
        let config = get_config().unwrap();
        App{
            mode: AppModes::RobotView,
            tf_listener: Arc::new(Mutex::new(TfListener::new())),
            listeners: Listeners::new(
                Arc::new(Mutex::new(TfListener::new())),
                config.fixed_frame,
                config.laser_topics,
                config.marker_array_topics,
                config.map_topics),
            terminal_size: terminal_size().unwrap(),
            zoom: 1.0,
            bounds: vec![-5., 5., -5., 5.],
            footprint_poly: get_footprint(),
            axis_length: config.axis_length,
        }
    }
}

impl App{
    pub fn init_terminal(&mut self) -> io::Result<Terminal<TermionBackend<AlternateScreen<MouseTerminal<RawTerminal<io::Stdout>>>>>>
    {
        let stdout = io::stdout().into_raw_mode()?;
        let stdout = MouseTerminal::from(stdout);
        let stdout = AlternateScreen::from(stdout);
        let backend = TermionBackend::new(stdout);
        let terminal = Terminal::new(backend)?;
        Ok(terminal)
    }
    fn calculate_footprint(
            ref_transform: &Arc<RwLock<rosrust_msg::geometry_msgs::Transform>>)
        -> Vec<(f64, f64, f64, f64)>
    {
        get_current_footprint(
            ref_transform,
            &get_footprint())
    }

    pub fn compute_bounds(
            &mut self,
            ref_transform: &Arc<RwLock<rosrust_msg::geometry_msgs::Transform>>)
        {
            let size_factor = self.terminal_size.0 as f64 / self.terminal_size.1 as f64 * 0.5;
            self.bounds = compute_bounds(ref_transform, self.zoom, size_factor);

        }

    pub fn increase_zoom(
        &mut self,
    )
    {
        self.zoom -= 0.1;
    }

    pub fn decrease_zoom(
        &mut self,
    )
    {
        self.zoom += 0.1;
    }

    pub fn draw_robot<B>(
            &mut self,
            f:&mut Frame<B>,
            tf: &Arc<RwLock<rosrust_msg::geometry_msgs::Transform>>)
        where B: Backend
    {
        let chunks = Layout::default()
            .constraints([Constraint::Percentage(100)].as_ref())
            .split(f.size());
        let canvas = Canvas::default()
            .block(
                Block::default()
                    .title(format!("Robot Viewer"))
                    .borders(Borders::NONE),
            )
            .x_bounds([self.bounds[0], self.bounds[1]])
            .y_bounds([self.bounds[2], self.bounds[3]])
            .paint(|ctx|{
                    for map in &self.listeners.maps {
                        ctx.draw(&Points {
                            coords: &map.points.read().unwrap(),
                            color: Color::Rgb(220, 220, 220),
                        });
                    }
                    ctx.layer();
                    for elem in App::calculate_footprint(tf) {
                        ctx.draw(&Line {
                            x1: elem.0,
                            y1: elem.1,
                            x2: elem.2,
                            y2: elem.3,
                            color: Color::Blue,
                        });
                    };
                    for laser in &self.listeners.lasers {
                        ctx.draw(&Points {
                            coords: &laser.points.read().unwrap(),
                            color: Color::Red,
                        });
                    }
                    for marker in &self.listeners.markers {
                        for line in marker.get_lines() {
                            ctx.draw(&line);
                        };
                    }
                    for line in get_frame_lines(tf, self.axis_length) { ctx.draw(&line); };
            }
            );
        f.render_widget(canvas, chunks[0]);

    }
}