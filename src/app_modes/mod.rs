//! A module that contains all the builing blocks to create app modes, as well as the app modes themselves.

pub mod image_view;
pub mod send_pose;
pub mod teleoperate;
pub mod topic_managment;
pub mod tf;
pub mod viewport;

use tui::backend::Backend;
use tui::Frame;

pub mod input {
    //! A module that contains the input actions that can be mapped to keys.

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
    pub const NEXT: &str = "Next";
    pub const PREVIOUS: &str = "Previous";
    pub const UPDATE: &str = "Update";
    pub const SHOW_HELP: &str = "Show help";
    pub const UNMAPPED: &str = "Any other";
}

/// Represents all the basic methods that an app mode must implement.
pub trait AppMode {
    /// Runs at each tick.
    fn run(&mut self);

    /// Runs when the mode is reset (e.g., if switching from another mode).
    fn reset(&mut self);

    /// Handles the received input.
    ///
    /// # Arguments
    /// - `input` : the input to be handled, in the form of app_modes::input
    fn handle_input(&mut self, input: &String);

    /// Returns a description of the mode as a vector of one String per line.
    fn get_description(&self) -> Vec<String>;

    /// Returns the key mapping that applies to this mode as a vector of arrays.
    /// Each array contains the input (from app_modes::input) and a description of what it does.
    fn get_keymap(&self) -> Vec<[String; 2]>;

    /// Returns the name of the mode.
    fn get_name(&self) -> String;
}

/// Represents something that can be drawn on the screen
pub trait Drawable<B: Backend> {
    /// Draws on the specified frame.
    ///
    /// # Arguments
    /// - `f`: the frame on which to draw
    fn draw(&self, f: &mut Frame<B>);
}

/// Represents the traits that any mode in termviz must implement
pub trait BaseMode<B: Backend>: AppMode + Drawable<B> {}
