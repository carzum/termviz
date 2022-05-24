pub mod image_view;
pub mod send_pose;
pub mod teleoperate;
pub mod viewport;

use tui::backend::Backend;
use tui::Frame;

pub mod input {
    pub const MODE_1: &str = "Switch to mode 1";
    pub const MODE_2: &str = "Switch to mode 2";
    pub const MODE_3: &str = "Switch to mode 3";
    pub const MODE_4: &str = "Switch to mode 4";
    pub const MODE_5: &str = "Switch to mode 5";
    pub const MODE_6: &str = "Switch to mode 6";
    pub const MODE_7: &str = "Switch to mode 7";
    pub const MODE_8: &str = "Switch to mode 8";
    pub const MODE_9: &str = "Switch to mode 9";
    pub const LEFT: &str = "Left";
    pub const RIGHT: &str = "Right";
    pub const UP: &str = "Up";
    pub const DOWN: &str = "Down";
    pub const ROTATE_LEFT: &str = "Counter-clockwise rotation";
    pub const ROTATE_RIGHT: &str = "Clockwise rotation";
    pub const CONFIRM: &str = "Confirm";
    pub const CANCEL: &str = "Cancel";
    pub const ZOOM_IN: &str = "Zoom in";
    pub const ZOOM_OUT: &str = "Zoom out";
    pub const INCREMENT_STEP: &str = "Increment step";
    pub const DECREMENT_STEP: &str = "Decrement step";
    pub const SHOW_HELP: &str = "Show help";
    pub const UNMAPPED: &str = "Any other";
}

pub trait AppMode {
    fn run(&mut self);
    fn reset(&mut self);
    fn handle_input(&mut self, input: &String);
    fn get_description(&self) -> Vec<String>;
    fn get_keymap(&self) -> Vec<[String; 2]>;
    fn get_name(&self) -> String;
}

pub trait Drawable<B: Backend> {
    fn draw(&self, f: &mut Frame<B>);
}

pub trait BaseMode<B: Backend>: AppMode + Drawable<B> {}
