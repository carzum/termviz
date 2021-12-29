use crate::config::get_config;
use crate::listeners::Listeners;
use crate::footprint::{get_footprint, get_current_footprint};
use crate::transformation;
use rosrust_msg;
use rustros_tf::TfListener;
use tui::Frame;
use tui::widgets::canvas::{Canvas, Line, Points};
use tui::widgets::{Block, Borders, Paragraph, Row, Table, Wrap};
use tui::backend::Backend;
use std::sync::{Arc, Mutex, RwLock};
use tui::style::{Color, Modifier, Style};
use tui::layout::{Alignment, Constraint, Direction, Layout};
use tui::text::{Span, Spans};
use termion::terminal_size;
use termion::input::MouseTerminal;
use termion::raw::IntoRawMode;
use termion::raw::RawTerminal;
use std::convert::TryFrom;
use std::io;
use termion::screen::AlternateScreen;
use tui::backend::TermionBackend;
use tui::Terminal;

pub enum AppModes{
    RobotView,
    HelpPage,
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
    listeners: Listeners,
    terminal_size: (u16, u16),
    initial_bounds: Vec<f64>,
    bounds: Vec<f64>,
    zoom: f64,
    axis_length: f64,
    zoom_factor: f64,
}

impl Default for App {
    fn default() -> Self{
        let config = get_config().unwrap();
        App{
            mode: AppModes::RobotView,
            listeners: Listeners::new(
                Arc::new(Mutex::new(TfListener::new())),
                config.fixed_frame,
                config.laser_topics,
                config.marker_array_topics,
                config.map_topics),
            terminal_size: terminal_size().unwrap(),
            zoom: 1.0,
            initial_bounds: config.visible_area.clone(),
            bounds: config.visible_area.clone(),
            axis_length: config.axis_length,
            zoom_factor: config.zoom_factor,
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
            // 0.5 is the height width ratio of terminal chars
            let scale_factor = self.terminal_size.0 as f64 / self.terminal_size.1 as f64 * 0.5;
            let tf = &ref_transform.as_ref().read().unwrap();
            self.bounds = vec![
                tf.translation.x + self.initial_bounds[0] / self.zoom * scale_factor,
                tf.translation.x + self.initial_bounds[1] / self.zoom * scale_factor,
                tf.translation.y + self.initial_bounds[2] / self.zoom,
                tf.translation.y + self.initial_bounds[3] / self.zoom,
            ];
        }

    pub fn increase_zoom(
        &mut self,
    )
    {
        self.zoom += self.zoom_factor;
    }

    pub fn decrease_zoom(
        &mut self,
    )
    {
        self.zoom -= self.zoom_factor;
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
            ["-", "Decreases the zoom."],
            ["=", "Increases the zoom."],
            ["h", "Shows this page."],
            [
                "k",
                "Increases the step size for manipulating the pose estimate.",
            ],
            [
                "j",
                "Decreases the step size for manipulating the pose estimate.",
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
                    Constraint::Min(u16::try_from(key_bindings_raw.len() + 2).unwrap()), // Table + header + space
                ]
                .as_ref(),
            )
            .split(f.size());

        // Conversion into tui stuff
        let key_bindings_rows = key_bindings_raw
            .iter()
            .map(|x| Row::Data(IntoIterator::into_iter(x)));

        let explanation_spans: std::vec::Vec<tui::text::Spans> =
            explanation_raw.into_iter().map(|x| Spans::from(Span::raw(x))).collect();

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

        let key_bindings = Table::new(
            IntoIterator::into_iter(["Key", "Function"]),
            IntoIterator::into_iter(key_bindings_rows),
        )
        .block(Block::default().title(" Key binding ").borders(Borders::ALL))
        .header_style(Style::default().fg(Color::Yellow))
        .widths(&[Constraint::Min(6), Constraint::Min(30)])
        .style(Style::default().fg(Color::White))
        .column_spacing(10);
        f.render_widget(title, areas[0]);
        f.render_widget(explanation, areas[1]);
        f.render_widget(key_bindings, areas[2]);
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
                    .title(format!("Robot View - Press h for help"))
                    .borders(Borders::NONE),
            )
            .x_bounds([self.bounds[0], self.bounds[1]])
            .y_bounds([self.bounds[2], self.bounds[3]])
            .paint(|ctx|{
                    for map in &self.listeners.maps {
                        ctx.draw(&Points {
                            coords: &map.points.read().unwrap(),
                            color: Color::Rgb(map.config.color.r,
                                              map.config.color.g,
                                              map.config.color.b),
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
                            color: Color::Rgb(laser.config.color.r,
                                              laser.config.color.g,
                                              laser.config.color.b),
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
