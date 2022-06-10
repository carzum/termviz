//! Image view mode allows to visualize images from the given topics.

use self::image::ImageListener;
use crate::app_modes::{input, AppMode, BaseMode, Drawable};
use crate::config::ImageListenerConfig;
use crate::image;
use tui::backend::Backend;
use tui::layout::{Alignment, Constraint, Layout};
use tui::style::{Color, Modifier, Style};
use tui::text::{Span, Spans};
use tui::widgets::{Block, Borders, Paragraph, Wrap};
use tui::Frame;
use tui_image::{ColorMode, Image};
pub struct ImageView {
    images: Vec<ImageListener>,
    active_sub: usize,
}


/// Represents the image view mode.
impl ImageView {
    pub fn new(image_topics: Vec<ImageListenerConfig>) -> ImageView {
        let mut images: Vec<image::ImageListener> = Vec::new();
        for image_config in image_topics {
            images.push(image::ImageListener::new(image_config));
        }
        ImageView {
            images: images,
            active_sub: 0,
        }
    }
}

impl AppMode for ImageView {
    fn run(&mut self) {
        if self.images.len() > 0 && !self.images[self.active_sub].is_active() {
            self.images[self.active_sub].activate();
        }
    }

    fn reset(&mut self) {
        for sub in self.images.iter_mut() {
            if sub.is_active() {
                sub.deactivate();
            }
        }
    }

    fn handle_input(&mut self, input: &String) {
        if self.images.len() > 0 {
            match input.as_str() {
                input::LEFT => {
                    self.images[self.active_sub].deactivate();
                    self.active_sub = if self.active_sub > 0 {
                        self.active_sub - 1
                    } else {
                        self.images.len() - 1
                    };
                }
                input::RIGHT => {
                    self.images[self.active_sub].deactivate();
                    self.active_sub = (self.active_sub + 1) % self.images.len();
                }
                input::ROTATE_RIGHT => {
                    self.images[self.active_sub].rotate(90);
                }
                input::ROTATE_LEFT => {
                    self.images[self.active_sub].rotate(-90);
                }
                _ => (),
            }
        }
    }

    fn get_description(&self) -> Vec<String> {
        vec!["This mode allows to visualized images received on the given topics.".to_string()]
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        vec![
            [
                input::LEFT.to_string(),
                "Switches to the previous image.".to_string(),
            ],
            [
                input::RIGHT.to_string(),
                "Switches to the next image.".to_string(),
            ],
            [
                input::ROTATE_LEFT.to_string(),
                "Rotates the image counter-clockwise.".to_string(),
            ],
            [
                input::ROTATE_RIGHT.to_string(),
                "Rotates the image clockwise.".to_string(),
            ],
        ]
    }

    fn get_name(&self) -> String {
        "Image".to_string()
    }
}

impl<B: Backend> Drawable<B> for ImageView {
    fn draw(&self, f: &mut Frame<B>) {
        let chunks = Layout::default()
            .constraints([Constraint::Length(1), Constraint::Percentage(100)].as_ref())
            .split(f.size());
        if self.images.len() == 0 {
            let header = Paragraph::new(Spans::from(Span::raw(
                self.get_name() + " view - No topic configured!",
            )))
            .block(Block::default().borders(Borders::NONE))
            .style(Style::default().fg(Color::White))
            .alignment(Alignment::Center)
            .wrap(Wrap { trim: false });
            f.render_widget(header, chunks[0]);
        } else {
            for image_sub in &self.images {
                if image_sub.is_active() {
                    let header = Paragraph::new(Spans::from(vec![
                        Span::styled(
                            self.get_name() + " view",
                            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
                        ),
                        Span::raw(" - Topic: /".to_string() + &image_sub.config.topic),
                    ]))
                    .block(Block::default().borders(Borders::NONE))
                    .style(Style::default().fg(Color::White))
                    .alignment(Alignment::Left)
                    .wrap(Wrap { trim: false });
                    f.render_widget(header, chunks[0]);
                    let image = image_sub.img.read().unwrap();
                    let widget = Image::with_img(image.clone()).color_mode(ColorMode::Rgb);
                    f.render_widget(widget, chunks[1]);
                    break;
                }
            }
        }
    }
}

impl<B: Backend> BaseMode<B> for ImageView {}
