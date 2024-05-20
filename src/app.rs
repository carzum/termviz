use crate::app_modes;
use crate::config::TermvizConfig;
use crate::footprint::get_footprint;
use crate::listeners::Listeners;
use crossterm::{
    event::EnableMouseCapture,
    execute,
    terminal::{enable_raw_mode, size, EnterAlternateScreen},
};
use std::cell::RefCell;
use std::collections::HashMap;
use std::convert::TryFrom;
use std::io;
use std::rc::Rc;
use std::sync::Arc;
use tui::backend::Backend;
use tui::backend::CrosstermBackend;
use tui::layout::{Alignment, Constraint, Direction, Layout};
use tui::style::{Color, Modifier, Style};
use tui::text::{Span, Spans};
use tui::widgets::{Block, Borders, Paragraph, Row, Table, Wrap};
use tui::{Frame, Terminal};

pub struct App<B: Backend> {
    mode: usize,
    show_help: bool,
    keymap: HashMap<String, String>,
    app_modes: Vec<Box<dyn app_modes::BaseMode<B>>>,
}

impl<B: Backend> App<B> {
    pub fn new(tf_listener: Arc<rustros_tf::TfListener>, config: TermvizConfig) -> App<B> {
        let config_copy = config.clone();
        let listeners = Listeners::new(
            tf_listener.clone(),
            config.fixed_frame.clone(),
            config.laser_topics,
            config.marker_topics,
            config.marker_array_topics,
            config.map_topics,
            config.pose_stamped_topics,
            config.pose_array_topics,
            config.pointcloud2_topics,
            config.polygon_stamped_topics,
            config.path_topics,
        );
        let viewport = Rc::new(RefCell::new(app_modes::viewport::Viewport::new(
            &config.fixed_frame,
            &config.robot_frame,
            tf_listener,
            &config.visible_area,
            &get_footprint(),
            config.axis_length,
            config.zoom_factor,
            listeners,
            size().unwrap(),
        )));
        let send_pose = Box::new(app_modes::send_pose::SendPose::new(
            &config.send_pose_topics,
            viewport.clone(),
        ));
        let teleop = Box::new(app_modes::teleoperate::Teleoperate::new(
            viewport.clone(),
            config.teleop,
        ));
        let topic_manager = Box::new(app_modes::topic_managment::TopicManager::new(config_copy));
        let tf_tree_view = Box::new(app_modes::tf::TfTreeView::new(viewport, config.tf_frames_service_name));

        let image_view = Box::new(app_modes::image_view::ImageView::new(config.image_topics));
        App {
            mode: 1,
            show_help: false,
            keymap: config.key_mapping,
            app_modes: vec![send_pose, teleop, image_view, topic_manager, tf_tree_view],
        }
    }

    pub fn init_terminal(&mut self) -> io::Result<Terminal<CrosstermBackend<io::Stdout>>> {
        enable_raw_mode()?;
        let mut stdout = io::stdout();
        execute!(stdout, EnterAlternateScreen, EnableMouseCapture)?;
        let backend = CrosstermBackend::new(stdout);
        let terminal = Terminal::new(backend)?;
        Ok(terminal)
    }

    pub fn run(&mut self) {
        self.app_modes[self.mode - 1].run();
    }

    pub fn draw(&self, f: &mut Frame<B>) {
        if self.show_help {
            self.show_help(f);
        } else {
            self.app_modes[self.mode - 1].draw(f);
        }
    }

    pub fn handle_input(&mut self, input: &String) {
        if input == app_modes::input::SHOW_HELP {
            if !self.show_help {
                self.show_help = true;
                self.app_modes[self.mode - 1].handle_input(input); // allows apps to do specific actions when help is called
            } else {
                self.show_help = false;
            }
        }
        let mut maybe_new_mode = None;
        match input.trim().parse::<usize>() {
            Ok(mode) => {
                maybe_new_mode = Some(mode);
            }
            Err(_e) => match input.as_str() {
                app_modes::input::MODE_1 => maybe_new_mode = Some(1),
                app_modes::input::MODE_2 => maybe_new_mode = Some(2),
                app_modes::input::MODE_3 => maybe_new_mode = Some(3),
                app_modes::input::MODE_4 => maybe_new_mode = Some(4),
                app_modes::input::MODE_5 => maybe_new_mode = Some(5),
                app_modes::input::MODE_6 => maybe_new_mode = Some(6),
                app_modes::input::MODE_7 => maybe_new_mode = Some(7),
                app_modes::input::MODE_8 => maybe_new_mode = Some(8),
                app_modes::input::MODE_9 => maybe_new_mode = Some(9),
                _ => {}
            },
        }
        match maybe_new_mode {
            Some(new_mode) => {
                if new_mode != self.mode && (1..self.app_modes.len() + 1).contains(&new_mode) {
                    self.app_modes[self.mode - 1].reset();
                    self.mode = new_mode;
                    self.app_modes[self.mode - 1].reset();
                    return;
                }
            }
            None => {}
        }
        if self.show_help {
            return;
        }
        self.app_modes[self.mode - 1].handle_input(input);
    }

    pub fn show_help(&self, f: &mut Frame<B>)
    where
        B: Backend,
    {
        // Text
        let mut key_bindings_raw: Vec<[String; 2]> = self
            .app_modes
            .iter()
            .enumerate()
            .map(|(i, mode)| {
                [
                    format!("Switch to mode {}", (i + 1)),
                    "Switches to ".to_string() + &mode.get_name() + &" mode.".to_string(),
                ]
            })
            .collect();
        key_bindings_raw.push(["".to_string(), "".to_string()]);
        key_bindings_raw.extend(self.app_modes[self.mode - 1].get_keymap());
        key_bindings_raw.extend([
            ["".to_string(), "".to_string()],
            [
                app_modes::input::SHOW_HELP.to_string(),
                "Opens/closes this page.".to_string(),
            ],
            ["Ctrl+c".to_string(), "Quits the application.".to_string()],
        ]);
        for e in &mut key_bindings_raw {
            match self.keymap.get(&e[0]) {
                Some(elem) => e[0] = elem.clone(),
                None => (),
            }
        }
        for i in 0..self.app_modes.len() {
            if key_bindings_raw[i][0].contains("Switch") {
                key_bindings_raw[i][0] = (i + 1).to_string();
            } else {
                key_bindings_raw[i][0] = (i + 1).to_string() + ", " + &key_bindings_raw[i][0];
            }
        }
        let title_text = vec![Spans::from(Span::styled(
            "TermViz - ".to_string() + &self.app_modes[self.mode - 1].get_name(),
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        ))];

        // Define areas from text
        let areas = Layout::default()
            .direction(Direction::Vertical)
            .horizontal_margin(20)
            .constraints(
                [
                    Constraint::Length(3), // Title + 2 borders
                    Constraint::Length(
                        u16::try_from(self.app_modes[self.mode - 1].get_description().len() + 2)
                            .unwrap(),
                    ), // Text + 2 borders
                    Constraint::Min(u16::try_from(key_bindings_raw.len() + 3).unwrap()), // Table + header + space
                ]
                .as_ref(),
            )
            .split(f.size());

        // Conversion into tui stuff
        let key_bindings_rows = key_bindings_raw.into_iter().map(|x| Row::new(x));

        let explanation_spans: std::vec::Vec<tui::text::Spans> = self.app_modes[self.mode - 1]
            .get_description()
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
            .widths(&[Constraint::Min(9), Constraint::Percentage(100)])
            .style(Style::default().fg(Color::White))
            .column_spacing(10);
        f.render_widget(title, areas[0]);
        f.render_widget(explanation, areas[1]);
        f.render_widget(key_bindings, areas[2]);
    }
}
