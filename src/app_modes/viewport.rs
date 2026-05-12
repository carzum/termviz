//! A viewport is where markers, maps and other information are shown.
//! A mode can borrow the viewport to draw whatever is needed.

use crate::app_modes::{input, AppMode, Drawable};
use crate::footprint::get_current_footprint;
use crate::listeners::Listeners;
use crate::ros;
use crate::ros::{tf::TfClient, types};
use crate::transformation::{self, iso2d_to_ros};
use nalgebra::Isometry2;
use std::sync::Arc;
use tui::backend::Backend;
use tui::layout::{Constraint, Layout};
use tui::style::{Color, Modifier, Style};
use tui::text::{Span, Spans};
use tui::widgets::canvas::{Canvas, Context, Line, Points};
use tui::widgets::{Block, Borders};
use tui::Frame;

/// Represents modes that use the viewport.
pub trait UseViewport: AppMode {
    /// Draws in the viewport
    ///
    /// # Arguments
    /// - `ctx`: the tui context where to draw.
    fn draw_in_viewport(&self, ctx: &mut Context);

    /// Returns the horizontal bounds of the window.
    /// Useful for panning/zooming the view.
    fn x_bounds(&self) -> [f64; 2];

    /// Returns the vertical bounds of the window.
    /// Useful for panning/zooming the view.
    fn y_bounds(&self) -> [f64; 2];

    /// Returns additional information that will be displayed on the top bar of the viewport.
    fn info(&self) -> String;
}

impl<B: Backend, T: UseViewport> Drawable<B> for T {
    fn draw(&self, f: &mut Frame<B>) {
        let chunks = Layout::default()
            .constraints([Constraint::Percentage(100)].as_ref())
            .split(f.size());

        let canvas = Canvas::default()
            .block(
                Block::default()
                    .title(Spans::from(vec![
                        Span::styled(
                            self.get_name(),
                            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
                        ),
                        Span::raw(" - "),
                        Span::raw(self.info()),
                    ]))
                    .borders(Borders::NONE),
            )
            .x_bounds(self.x_bounds())
            .y_bounds(self.y_bounds())
            .paint(|ctx| {
                self.draw_in_viewport(ctx);
            });
        f.render_widget(canvas, chunks[0]);
    }
}

pub struct Viewport {
    pub static_frame: String,
    pub robot_frame: String,
    pub tf: Arc<dyn TfClient>,
    pub initial_bounds: Vec<f64>,
    pub footprint: Vec<(f64, f64)>,
    pub axis_length: f64,
    pub zoom: f64,
    pub zoom_factor: f64,
    pub terminal_size: (u16, u16),
    pub listeners: Listeners, // TODO split properly config and listeners
}

impl Viewport {
    pub fn new(
        static_frame: &String,
        robot_frame: &String,
        tf: Arc<dyn TfClient>,
        initial_bounds: &Vec<f64>,
        footprint: &Vec<(f64, f64)>,
        axis_length: f64,
        zoom_factor: f64,
        listeners: Listeners,
        terminal_size: (u16, u16),
    ) -> Viewport {
        Viewport {
            static_frame: static_frame.clone(),
            robot_frame: robot_frame.clone(),
            tf,
            initial_bounds: initial_bounds.clone(),
            zoom: 1.0,
            zoom_factor: zoom_factor,
            footprint: footprint.clone(),
            axis_length: axis_length,
            listeners: listeners,
            terminal_size: terminal_size,
        }
    }
    pub fn get_frame_lines(
        tf: &types::Transform,
        axis_length: f64,
    ) -> Vec<Line> {
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
}

impl AppMode for Viewport {
    fn run(&mut self) {}
    fn reset(&mut self) {}
    fn handle_input(&mut self, input: &String) {
        match input.as_str() {
            input::ZOOM_IN => self.zoom += self.zoom_factor,
            input::ZOOM_OUT => self.zoom -= self.zoom_factor,
            _ => return,
        }
    }

    fn get_name(&self) -> String {
        "".to_string()
    }

    fn get_description(&self) -> Vec<String> {
        vec![]
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        vec![
            [
                input::ZOOM_IN.to_string(),
                "Increases the zoom.".to_string(),
            ],
            [
                input::ZOOM_OUT.to_string(),
                "Decreases the zoom.".to_string(),
            ],
        ]
    }
}

impl UseViewport for Viewport {
    fn x_bounds(&self) -> [f64; 2] {
        let scale_factor = self.terminal_size.0 as f64 / self.terminal_size.1 as f64 * 0.5;
        let res = self
            .tf
            .lookup_transform(&self.static_frame, &self.robot_frame, ros::now());
        match &res {
            Ok(res) => res,
            Err(_e) => {
                return [
                    self.initial_bounds[0] / self.zoom * scale_factor,
                    self.initial_bounds[1] / self.zoom * scale_factor,
                ]
            }
        };
        let tf = res.as_ref().unwrap();

        [
            tf.transform.translation.x + self.initial_bounds[0] / self.zoom * scale_factor,
            tf.transform.translation.x + self.initial_bounds[1] / self.zoom * scale_factor,
        ]
    }
    fn y_bounds(&self) -> [f64; 2] {
        let scale_factor = self.terminal_size.0 as f64 / self.terminal_size.1 as f64 * 0.5;
        let res = self
            .tf
            .lookup_transform(&self.static_frame, &self.robot_frame, ros::now());
        match &res {
            Ok(res) => res,
            Err(_e) => {
                return [
                    self.initial_bounds[2] / self.zoom * scale_factor,
                    self.initial_bounds[3] / self.zoom * scale_factor,
                ]
            }
        };
        let tf = res.as_ref().unwrap();
        [
            tf.transform.translation.y + self.initial_bounds[2] / self.zoom,
            tf.transform.translation.y + self.initial_bounds[3] / self.zoom,
        ]
    }

    fn info(&self) -> String {
        "".to_string()
    }
    fn draw_in_viewport(&self, ctx: &mut Context) {
        for map in &self.listeners.maps {
            ctx.draw(&Points {
                coords: &map.points.read().unwrap(),
                color: Color::Rgb(map.config.color.r, map.config.color.g, map.config.color.b),
            });
        }

        ctx.layer();
        for pointcloud in &self.listeners.pointclouds {
            let points = &pointcloud.points.read().unwrap().clone();
            for pt in points {
                ctx.draw(&Points {
                    coords: &[(pt.point.x, pt.point.y)],
                    color: pt.color,
                })
            }
        }

        ctx.layer();
        for line in self.listeners.markers.get_lines() {
            ctx.draw(&line);
        }

        ctx.layer();
        for laser in &self.listeners.lasers {
            ctx.draw(&Points {
                coords: &laser.points.read().unwrap(),
                color: Color::Rgb(
                    laser.config.color.r,
                    laser.config.color.g,
                    laser.config.color.b,
                ),
            });
        }

        ctx.layer();
        let base_link_pose =
            self.tf
                .lookup_transform(&self.static_frame, &self.robot_frame, ros::now());

        let robot_pose = if base_link_pose.is_ok() {
            base_link_pose.unwrap().transform
        } else {
            iso2d_to_ros(&Isometry2::identity())
        };
        get_current_footprint(&robot_pose, &self.footprint);

        for elem in get_current_footprint(&robot_pose, &self.footprint) {
            ctx.draw(&Line {
                x1: elem.0,
                y1: elem.1,
                x2: elem.2,
                y2: elem.3,
                color: Color::Blue,
            });
        }

        for line in Viewport::get_frame_lines(&robot_pose, self.axis_length) {
            ctx.draw(&line);
        }

        for pose_stamped in &self.listeners.pose_stamped {
            for line in pose_stamped.get_lines() {
                ctx.draw(&line);
            }
        }

        for polygon in &self.listeners.polygons {
            for line in polygon.get_lines() {
                ctx.draw(&line);
            }
        }

        for path in &self.listeners.paths {
            for line in path.get_lines() {
                ctx.draw(&line)
            }
        }

        for pose_array in &self.listeners.pose_array {
            for line in pose_array.get_lines() {
                ctx.draw(&line);
            }
        }

        ctx.layer();
    }
}
