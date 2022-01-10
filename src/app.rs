use crate::config::get_config;
use crate::footprint::{get_current_footprint, get_footprint};
use crate::listeners::Listeners;
use crate::rosout;
use crate::transformation;
use ansi_to_tui::ansi_to_text;
use nalgebra::{Isometry2, Vector2};
use std::convert::TryFrom;
use std::io;
use std::sync::Arc;
use strum_macros::AsRefStr;
use termion::input::MouseTerminal;
use termion::raw::IntoRawMode;
use termion::raw::RawTerminal;
use termion::screen::AlternateScreen;
use termion::terminal_size;
use tui::backend::Backend;
use tui::backend::TermionBackend;
use tui::layout::{Alignment, Constraint, Direction, Layout, Rect};
use tui::style::{Color, Modifier, Style};
use tui::text::{Span, Spans, Text};
use tui::widgets::canvas::{Canvas, Line, Points};
use tui::widgets::{Block, Borders, Paragraph, Row, Table, Wrap};
use tui::{Frame, Terminal};

#[derive(PartialEq, AsRefStr)]
pub enum AppModes {
    RobotView,
    SendPose,
    HelpPage,
    Teleoperate,
}

pub fn get_frame_lines(tf: &rosrust_msg::geometry_msgs::Transform, axis_length: f64) -> Vec<Line> {
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

pub struct App {
    pub mode: AppModes,
    terminal_size: (u16, u16),
    initial_bounds: Vec<f64>,
    bounds: Vec<f64>,
    zoom: f64,
    axis_length: f64,
    zoom_factor: f64,
    static_frame: String,
    robot_frame: String,
    listeners: Listeners,
    footprint: Vec<(f64, f64)>,
    initial_pose: Isometry2<f64>,
    pose_estimate: Isometry2<f64>,
    pub rosout_listener: rosout::RosoutListener,
    rosout_screen_percentage: u16,
    pub rosout_widget_enabled: bool,
}

impl App {
    pub fn new(tf_listener: Arc<rustros_tf::TfListener>) -> App {
        let config = get_config().unwrap();
        let listeners = Listeners::new(
            tf_listener.clone(),
            config.fixed_frame.clone(),
            config.laser_topics,
            config.marker_array_topics,
            config.map_topics,
        );
        let base_link_pose = tf_listener
            .lookup_transform(
                &config.fixed_frame,
                &config.robot_frame,
                rosrust::Time::new(),
            )
            .unwrap()
            .transform;
        let initial_pose = transformation::ros_to_iso2d(&base_link_pose);
        App {
            axis_length: config.axis_length,
            bounds: config.visible_area.clone(),
            initial_bounds: config.visible_area.clone(),
            mode: AppModes::RobotView,
            terminal_size: terminal_size().unwrap(),
            zoom: 1.0,
            zoom_factor: config.zoom_factor,
            static_frame: config.fixed_frame,
            robot_frame: config.robot_frame,
            footprint: get_footprint(),
            listeners: listeners,
            initial_pose: initial_pose.clone(),
            pose_estimate: initial_pose,
            rosout_listener: rosout::RosoutListener::new(
                config.rosout_config.buffer_size,
                config.rosout_config.enabled_by_default,
                config.rosout_config.min_loglevel,
            ),
            rosout_screen_percentage: config.rosout_config.screen_percentage,
            rosout_widget_enabled: config.rosout_config.enabled_by_default,
        }
    }

    pub fn init_terminal(
        &mut self,
    ) -> io::Result<Terminal<TermionBackend<AlternateScreen<MouseTerminal<RawTerminal<io::Stdout>>>>>>
    {
        let stdout = io::stdout().into_raw_mode()?;
        let stdout = MouseTerminal::from(stdout);
        let stdout = AlternateScreen::from(stdout);
        let backend = TermionBackend::new(stdout);
        let terminal = Terminal::new(backend)?;
        Ok(terminal)
    }

    pub fn compute_bounds(&mut self, tf_listener: Arc<rustros_tf::TfListener>) {
        // 0.5 is the height width ratio of terminal chars
        let scale_factor = self.terminal_size.0 as f64 / self.terminal_size.1 as f64 * 0.5;
        let res = tf_listener.clone().lookup_transform(
            &self.static_frame,
            &self.robot_frame,
            rosrust::Time::new(),
        );
        match &res {
            Ok(res) => res,
            Err(_e) => return,
        };
        let tf = res.as_ref().unwrap();

        self.bounds = vec![
            tf.transform.translation.x + self.initial_bounds[0] / self.zoom * scale_factor,
            tf.transform.translation.x + self.initial_bounds[1] / self.zoom * scale_factor,
            tf.transform.translation.y + self.initial_bounds[2] / self.zoom,
            tf.transform.translation.y + self.initial_bounds[3] / self.zoom,
        ];
    }

    pub fn increase_zoom(&mut self) {
        self.zoom += self.zoom_factor;
    }

    pub fn decrease_zoom(&mut self) {
        self.zoom -= self.zoom_factor;
    }

    pub fn move_pose_estimate(&mut self, x: f64, y: f64, yaw: f64) {
        let new_yaw = self.pose_estimate.rotation.angle() + yaw;
        let new_x = x * new_yaw.cos() - y * new_yaw.sin() + self.pose_estimate.translation.x;
        let new_y = x * new_yaw.sin() + y * new_yaw.cos() + self.pose_estimate.translation.y;
        self.pose_estimate = Isometry2::new(Vector2::new(new_x, new_y), new_yaw);
    }

    pub fn get_pose_estimate(&self) -> rosrust_msg::geometry_msgs::Transform {
        transformation::iso2d_to_ros(&self.pose_estimate)
    }
    pub fn show_help<B>(&mut self, f: &mut Frame<B>)
    where
        B: Backend,
    {
        // Text
        let key_bindings_raw = vec![
            ["w", "Shifts the pose estimate positively along the x axis."],
            ["s", "Shifts the pose estimate negatively along the x axis."],
            ["d", "Shifts the pose estimate positively along the y axis."],
            ["a", "Shifts the pose estimate negatively along the y axis."],
            ["q", "Rotates the pose estimate counter-clockwise."],
            ["e", "Rotates the pose estimate clockwise."],
            ["t", "Enter/Exit teleoperating mode"],
            ["Esc", "Resets the pose estimate."],
            ["Enter", "Sends the pose estimate."],
            ["-", "Decreases the zoom."],
            ["=", "Increases the zoom."],
            [
                "k",
                "Increases the step size for manipulating the pose estimate.",
            ],
            [
                "j",
                "Decreases the step size for manipulating the pose estimate.",
            ],
            [
                "L",
                "Switches the mode of the rosout log display: 
                   enabled -> stopped -> disabled -> enabled ...",
            ],
            ["h", "Shows this page."],
            ["Ctrl+c", "Quits the application."],
        ];
        let explanation_raw = vec![
            "", // Leave some space from the top
            "Welcome to TermViz!",
            "",
            "To get started, take a look at the configuration file, which is located in ~/.config/termviz/termviz.yml.",
            "",
            "Press 't' to enter/exit teleoperating mode, where the configured keys (default wasd qe) are used to move the robot. \
            When entering the mode, a zero velocity vector is sent, stopping the robot. \
            Additionally, any button but the configured ones will stop the robot.",
            "",
            "Press any key to go back to the robot view, or Ctrl+c to exit.",
            "", // Leave some space to the bottom
        ];
        let title_text = vec![Spans::from(Span::styled(
            "TermViz",
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        ))];

        // Define areas from text
        let areas = Layout::default()
            .direction(Direction::Vertical)
            .horizontal_margin(20)
            .constraints(
                [
                    Constraint::Length(3), // Title + 2 borders
                    Constraint::Min(u16::try_from(explanation_raw.len() + 2).unwrap()), // Text + 2 borders
                    Constraint::Min(u16::try_from(key_bindings_raw.len() + 3).unwrap()), // Table + header + space
                ]
                .as_ref(),
            )
            .split(f.size());

        // Conversion into tui stuff
        let key_bindings_rows = key_bindings_raw.into_iter().map(|x| Row::new(x));

        let explanation_spans: std::vec::Vec<tui::text::Spans> = explanation_raw
            .into_iter()
            .map(|x| Spans::from(Span::raw(x)))
            .collect();

        // Widget creation
        let title = Paragraph::new(title_text)
            .block(Block::default().borders(Borders::ALL))
            .style(Style::default().fg(Color::White))
            .alignment(Alignment::Center)
            .wrap(Wrap { trim: false });

        let explanation = Paragraph::new(explanation_spans)
            .block(Block::default().borders(Borders::ALL))
            .style(Style::default().fg(Color::White))
            .alignment(Alignment::Center)
            .wrap(Wrap { trim: false });

        let key_bindings = Table::new(IntoIterator::into_iter(key_bindings_rows))
            .block(
                Block::default()
                    .title(" Key binding ")
                    .borders(Borders::ALL),
            )
            .header(Row::new(vec!["Key", "Function"]).style(Style::default().fg(Color::Yellow)))
            .widths(&[Constraint::Min(6), Constraint::Min(30)])
            .style(Style::default().fg(Color::White))
            .column_spacing(10);
        f.render_widget(title, areas[0]);
        f.render_widget(explanation, areas[1]);
        f.render_widget(key_bindings, areas[2]);
    }

    pub fn draw_robot<B>(&mut self, f: &mut Frame<B>, tf_listener: Arc<rustros_tf::TfListener>)
    where
        B: Backend,
    {
        let chunks: Vec<Rect>;
        if self.rosout_widget_enabled {
            assert!(self.rosout_screen_percentage <= 100);
            let robot_view_percentage = 100 - self.rosout_screen_percentage;
            chunks = Layout::default()
                .constraints(
                    [
                        Constraint::Percentage(robot_view_percentage),
                        Constraint::Percentage(self.rosout_screen_percentage),
                    ]
                    .as_ref(),
                )
                .split(f.size());
        } else {
            chunks = Layout::default()
                .constraints([Constraint::Percentage(100)].as_ref())
                .split(f.size());
        }

        let base_link_pose = tf_listener
            .lookup_transform(&self.static_frame, &self.robot_frame, rosrust::Time::new())
            .unwrap()
            .transform;
        self.initial_pose = transformation::ros_to_iso2d(&base_link_pose);
        if matches!(self.mode, AppModes::RobotView) {
            self.pose_estimate = self.initial_pose.clone();
        }
        let footprint = get_current_footprint(&base_link_pose, &self.footprint);

        let canvas = Canvas::default()
            .block(
                Block::default()
                    .title(format!("{} - Press h for help", self.mode.as_ref()))
                    .borders(Borders::NONE),
            )
            .x_bounds([self.bounds[0], self.bounds[1]])
            .y_bounds([self.bounds[2], self.bounds[3]])
            .paint(|ctx| {
                for map in &self.listeners.maps {
                    ctx.draw(&Points {
                        coords: &map.points.read().unwrap(),
                        color: Color::Rgb(
                            map.config.color.r,
                            map.config.color.g,
                            map.config.color.b,
                        ),
                    });
                }
                ctx.layer();
                for elem in &footprint {
                    ctx.draw(&Line {
                        x1: elem.0,
                        y1: elem.1,
                        x2: elem.2,
                        y2: elem.3,
                        color: Color::Blue,
                    });
                }
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
                for marker in &self.listeners.markers {
                    for line in marker.get_lines() {
                        ctx.draw(&line);
                    }
                }

                for line in get_frame_lines(&base_link_pose, self.axis_length) {
                    ctx.draw(&line);
                }
                if matches!(self.mode, AppModes::SendPose) {
                    let pose_estimate_ros = transformation::iso2d_to_ros(&self.pose_estimate);
                    for elem in &get_current_footprint(&pose_estimate_ros, &self.footprint) {
                        ctx.draw(&Line {
                            x1: elem.0,
                            y1: elem.1,
                            x2: elem.2,
                            y2: elem.3,
                            color: Color::Gray,
                        });
                    }
                    for mut line in get_frame_lines(&pose_estimate_ros, self.axis_length) {
                        line.color = Color::Gray;
                        ctx.draw(&line);
                    }
                }
            });
        f.render_widget(canvas, chunks[0]);
        if self.rosout_widget_enabled {
            f.render_widget(self.build_rosout_widget(), chunks[1]);
        }
    }

    pub fn toggle_rosout_widget(&mut self) {
        if self.rosout_listener.is_buffering() {
            self.rosout_listener.toggle_buffering();
        } else if self.rosout_widget_enabled {
            self.rosout_widget_enabled = !self.rosout_widget_enabled;
        } else {
            self.rosout_listener.toggle_buffering();
            self.rosout_widget_enabled = true;
        }
    }

    pub fn build_rosout_widget(&mut self) -> Paragraph {
        let logstrings = self.rosout_listener.read_logstring_buffer();
        let mut all_spans = Vec::<Spans>::new();
        for logstring in logstrings.iter() {
            all_spans.extend(ansi_to_text(logstring.as_bytes().to_vec()).unwrap().lines);
        }
        return Paragraph::new(Text::from(all_spans))
            .block(Block::default().borders(Borders::ALL).title("rosout"));
    }
}
