use std::cell::RefCell;
use std::collections::{HashMap, HashSet};
use std::rc::Rc;
use std::time;

use crate::app_modes::viewport::Viewport as AppViewport;
use crate::app_modes::{input, AppMode, BaseMode, Drawable};

use nalgebra::geometry::{Quaternion, UnitQuaternion};
use serde::Deserialize;
use tui::backend::Backend;
use tui::layout::{Alignment, Constraint, Direction, Layout};
use tui::style::{Color, Modifier, Style};
use tui::text::{Span, Spans, Text};
use tui::widgets::{Block, Borders, Paragraph, Wrap};
use tui::Frame;
use tui_tree_widget::{flatten, get_identifier_without_leaf, Tree, TreeItem, TreeState};

/// Struct that represents tf node that is being returned by tf2_frames service
#[derive(Debug, Deserialize, Clone)]
struct Node {
    parent: String,
    #[allow(dead_code)]
    broadcaster: String,
    rate: f64,
    most_recent_transform: f64,
    oldest_transform: f64,
    #[allow(dead_code)]
    buffer_length: f64,
}

/// Struct that represents internal tree node with children
#[derive(Debug)]
struct TreeNode {
    name: String,
    children: Vec<TreeNode>,
}

/// Find unconnected roots from dictionary of nodes
fn find_roots(nodes: &HashMap<String, Node>) -> Vec<String> {
    let mut parents = HashSet::new();
    let mut children = HashSet::new();

    for (name, node) in nodes {
        parents.insert(node.parent.clone());
        children.insert(name.clone());
    }

    parents.difference(&children).cloned().collect()
}

/// Given dictionary of nodes construct simple Tree structure from it
fn build_tree(nodes: &HashMap<String, Node>, root_name: &str) -> TreeNode {
    let mut tree_node = TreeNode {
        name: root_name.to_string(),
        children: Vec::new(),
    };

    for (name, node) in nodes {
        if node.parent == root_name {
            tree_node.children.push(build_tree(nodes, name));
        }
    }

    tree_node
}

/// Convert internal TreeNode to TreeItem class that is used by tui_tree_widget
fn convert_to_tree_item(tree_node: &TreeNode) -> TreeItem<'static> {
    if tree_node.children.is_empty() {
        TreeItem::new_leaf(tree_node.name.clone())
    } else {
        TreeItem::new(
            tree_node.name.clone(),
            tree_node
                .children
                .iter()
                .map(convert_to_tree_item)
                .collect::<Vec<TreeItem<'static>>>(),
        )
    }
}

/// Parse string from tui::Text
fn text_to_string(text: &Text) -> String {
    let mut result = String::new();

    for line in &text.lines {
        for span in &line.0 {
            result.push_str(&span.content);
        }
    }

    result
}

/// TreeItems with state(opened, closed)
pub struct StatefulTree<'a> {
    pub state: TreeState,
    pub items: Vec<TreeItem<'a>>,
}

impl<'a> StatefulTree<'a> {
    #[allow(dead_code)]
    pub fn new() -> Self {
        Self {
            state: TreeState::default(),
            items: Vec::new(),
        }
    }

    pub fn with_items(items: Vec<TreeItem<'a>>) -> Self {
        Self {
            state: TreeState::default(),
            items,
        }
    }

    fn move_up_down(&mut self, down: bool) {
        let visible = flatten(&self.state.get_all_opened(), &self.items);
        let current_identifier = self.state.selected();
        let current_index = visible
            .iter()
            .position(|o| o.identifier == current_identifier);
        let new_index = current_index.map_or(0, |current_index| {
            if down {
                current_index.saturating_add(1)
            } else {
                current_index.saturating_sub(1)
            }
            .min(visible.len() - 1)
        });

        match visible.get(new_index) {
            Some(new_idx) => {
                self.state.select(new_idx.identifier.clone());
                Some(new_idx.identifier.clone())
            }
            None => return,
        };
    }

    pub fn next(&mut self) {
        self.move_up_down(true);
    }

    pub fn previous(&mut self) {
        self.move_up_down(false);
    }

    pub fn close(&mut self) {
        let selected = self.state.selected();
        if !self.state.close(&selected) {
            let (head, _) = get_identifier_without_leaf(&selected);
            self.state.select(head);
        }
    }

    pub fn open(&mut self) {
        self.state.open(self.state.selected());
    }

    pub fn choose(&mut self) {
        self.state.pick(self.state.selected());
    }

    pub fn get_picked_item(&self) -> Text {
        let visible = flatten(&self.state.get_all_opened(), &self.items);
        let current_identifier = self.state.picked();

        match visible
            .iter()
            .position(|o| o.identifier == current_identifier)
        {
            Some(current_index) => {
                if let Some(item) = visible.get(current_index) {
                    return item.item.text.clone();
                }
            }
            None => {}
        }

        Text::from(Spans::from(Span::raw("")))
    }

    pub fn get_selected_item(&self) -> Text {
        let visible = flatten(&self.state.get_all_opened(), &self.items);
        let current_identifier = self.state.selected();

        match visible
            .iter()
            .position(|o| o.identifier == current_identifier)
        {
            Some(current_index) => {
                if let Some(item) = visible.get(current_index) {
                    return item.item.text.clone();
                }
            }
            None => {}
        }

        Text::from(Spans::from(Span::raw("")))
    }
}

// Struct to hold transformation data for display
#[derive(Debug, Clone)]
struct TransformationDetails {
    parent_frame: String,
    child_frame: String,
    translation: (f64, f64, f64),
    rotation_quaternion: (f64, f64, f64, f64),
    rotation_euler: (f64, f64, f64),
}

impl TransformationDetails {
    fn from_tf(tf_stamped: rosrust_msg::geometry_msgs::TransformStamped) -> TransformationDetails {
        let tf = tf_stamped.transform;
        let rot = UnitQuaternion::new_normalize(Quaternion::new(
            tf.rotation.w,
            tf.rotation.x,
            tf.rotation.y,
            tf.rotation.z,
        ));
        let (roll, pitch, yaw) = rot.euler_angles();

        TransformationDetails {
            parent_frame: tf_stamped.header.frame_id.clone(),
            child_frame: tf_stamped.child_frame_id.clone(),
            translation: (tf.translation.x, tf.translation.y, tf.translation.z),
            rotation_quaternion: (tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w),
            rotation_euler: (roll, pitch, yaw),
        }
    }

    fn to_spans(&self) -> Vec<Spans<'static>> {
        vec![
            Spans::from(Span::styled(
                format!("Parent frame: {}", self.parent_frame),
                Style::default().add_modifier(Modifier::ITALIC),
            )),
            Spans::from(Span::styled(
                format!("Child frame: {}", self.child_frame),
                Style::default().add_modifier(Modifier::ITALIC),
            )),
            Spans::from(Span::styled(
                "Translation",
                Style::default().add_modifier(Modifier::BOLD),
            )),
            Spans::from(Span::styled(
                format!(" X: {:.5}", self.translation.0),
                Style::default(),
            )),
            Spans::from(Span::styled(
                format!(" Y: {:.5}", self.translation.1),
                Style::default(),
            )),
            Spans::from(Span::styled(
                format!(" Z: {:.5}", self.translation.2),
                Style::default(),
            )),
            Spans::from(Span::styled(
                "Rotation Quaternion",
                Style::default().add_modifier(Modifier::BOLD),
            )),
            Spans::from(Span::styled(
                format!(" X: {:.5}", self.rotation_quaternion.0),
                Style::default(),
            )),
            Spans::from(Span::styled(
                format!(" Y: {:.5}", self.rotation_quaternion.1),
                Style::default(),
            )),
            Spans::from(Span::styled(
                format!(" Z: {:.5}", self.rotation_quaternion.2),
                Style::default(),
            )),
            Spans::from(Span::styled(
                format!(" W: {:.5}", self.rotation_quaternion.3),
                Style::default(),
            )),
            Spans::from(Span::styled(
                "Rotation Euler",
                Style::default().add_modifier(Modifier::BOLD),
            )),
            Spans::from(Span::styled(
                format!(" Roll: {:.5}", self.rotation_euler.0),
                Style::default(),
            )),
            Spans::from(Span::styled(
                format!(" Pitch: {:.5}", self.rotation_euler.1),
                Style::default(),
            )),
            Spans::from(Span::styled(
                format!(" Yaw: {:.5}", self.rotation_euler.2),
                Style::default(),
            )),
        ]
    }
}

/// TfTreeView displays TF tree and allows user to get transform between two arbitrary frames.
pub struct TfTreeView<'a> {
    nodes: HashMap<String, Node>,
    status_bar: String,
    tf_client: Option<rosrust::Client<rosrust_msg::tf2_msgs::FrameGraph>>,
    tf_echo_details: Vec<Spans<'static>>,
    tf_frames_service_name: String,
    tree: StatefulTree<'a>,
    updated_once: bool,
    viewport: Rc<RefCell<AppViewport>>,
}

impl<'a> TfTreeView<'a> {
    const CONNECTION_ERROR_MSG: &'static str = "Can't establish connection to service";
    const DEFAULT_MSG: &'static str = "Select frame to view its data";

    fn update_status_bar_for_error(&mut self) {
        self.status_bar = format!(
            "{} {}",
            Self::CONNECTION_ERROR_MSG,
            self.tf_frames_service_name
        );
    }

    fn update_status_bar_for_selection(&mut self) {
        self.status_bar = Self::DEFAULT_MSG.to_string();
    }

    pub fn new(
        viewport: Rc<RefCell<AppViewport>>,
        tf_frames_service_name: String,
    ) -> TfTreeView<'a> {
        let mut updated_once = false;
        let mut status_bar = format!("{} {}", Self::CONNECTION_ERROR_MSG, tf_frames_service_name);

        let tf_client = if rosrust::wait_for_service(
            tf_frames_service_name.as_str(),
            Some(time::Duration::from_secs(1)),
        )
        .is_ok()
        {
            status_bar = Self::DEFAULT_MSG.to_string();
            Some(
                rosrust::client::<rosrust_msg::tf2_msgs::FrameGraph>(
                    tf_frames_service_name.as_str(),
                )
                .unwrap(),
            )
        } else {
            updated_once = true;
            None
        };

        TfTreeView {
            nodes: HashMap::default(),
            status_bar,
            tf_client,
            tf_echo_details: vec![Spans::from(format!(
                "Select frames with {} button to view transform",
                input::CONFIRM
            ))],
            tf_frames_service_name: tf_frames_service_name.clone(),
            tree: StatefulTree::with_items(vec![]),
            updated_once,
            viewport,
        }
    }

    fn update_tf_client(&mut self) {
        if self.tf_client.is_some() {
            return;
        }

        if rosrust::wait_for_service(
            &self.tf_frames_service_name,
            Some(time::Duration::from_secs(1)),
        )
        .is_ok()
        {
            self.tf_client =
                rosrust::client::<rosrust_msg::tf2_msgs::FrameGraph>(&self.tf_frames_service_name)
                    .ok();
            self.update_status_bar_for_selection();
        } else {
            self.update_status_bar_for_error();
        }
    }

    /// Process tf2_frames service result and update tf tree
    fn process_frame_data(&mut self) {
        let frame_request = rosrust_msg::tf2_msgs::FrameGraphReq {};
        let frame_response = match self.tf_client.as_ref().unwrap().req(&frame_request) {
            Ok(res) => res.unwrap(),
            Err(_) => return self.update_status_bar_for_error(),
        };

        match serde_yaml::from_str::<HashMap<String, Node>>(frame_response.frame_yaml.as_str()) {
            Ok(nodes) => {
                self.nodes = nodes;
                let mut tree_items = Vec::new();

                let roots = find_roots(&self.nodes);
                for root in roots {
                    let tree_node = build_tree(&self.nodes, &root);
                    let tree_item = convert_to_tree_item(&tree_node);
                    tree_items.push(tree_item);
                }

                self.tree = StatefulTree::with_items(tree_items);
                self.updated_once = true;
            }
            Err(_) => self.status_bar = "Failed to parse frame data".to_string(),
        }
    }

    /// Get transform between 'selected' and picked frame and store for display
    /// 'selected' means frame that is currently under cursor(focused), but it is name selected in
    /// tui_tree_widget library, so ambuqity kept consistent
    /// picked frame the one that user picked with button
    fn echo_selected_and_picked(&mut self) {
        let picked_item = text_to_string(&self.tree.get_picked_item());
        let selected_item = text_to_string(&self.tree.get_selected_item());

        self.tf_echo_details = match self.viewport.borrow().tf_listener.lookup_transform(
            &picked_item,
            &selected_item,
            rosrust::Time::new(),
        ) {
            Ok(tf_stamped) => TransformationDetails::from_tf(tf_stamped.clone()).to_spans(),
            Err(_) => vec![
                Spans::from(Span::raw(format!("Parent frame: {}", &picked_item))),
                Spans::from(Span::raw(format!("Child frame: {}", &selected_item))),
                Spans::from(Span::styled(
                    "Couldn't find transform",
                    Style::default().fg(Color::Red),
                )),
            ],
        };
        self.tree.choose();
    }
}

impl<'a, B: Backend> BaseMode<B> for TfTreeView<'a> {}

impl<'a> AppMode for TfTreeView<'a> {
    fn run(&mut self) {
        let selected_text = self.tree.get_selected_item();
        let string_representation = text_to_string(&selected_text);

        if let Some(node) = self.nodes.get(string_representation.as_str()) {
            self.status_bar = format!(
                "rate: {:e}, most_recent_tf: {}, oldest_tf: {}",
                node.rate, node.most_recent_transform, node.oldest_transform
            );
        }

        if self.updated_once {
            return;
        }

        self.update_tf_client();
        self.process_frame_data();
    }

    fn reset(&mut self) {}

    fn get_description(&self) -> Vec<String> {
        vec!["This mode displays tf tree view".to_string()]
    }

    fn handle_input(&mut self, input: &String) {
        match input.as_str() {
            input::UP => self.tree.previous(),
            input::DOWN => self.tree.next(),
            input::LEFT => self.tree.close(),
            input::RIGHT => self.tree.open(),
            input::UPDATE => {
                self.update_tf_client();
                self.process_frame_data();
            }
            input::CONFIRM => self.echo_selected_and_picked(),
            _ => (),
        }
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        vec![
            [input::UPDATE.to_string(), "Update TF tree".to_string()],
            [
                input::UP.to_string(),
                "Selects the previous frame".to_string(),
            ],
            [
                input::DOWN.to_string(),
                "Selects the next frame".to_string(),
            ],
            [input::RIGHT.to_string(), "Expand frame subtree".to_string()],
            [
                input::LEFT.to_string(),
                "Collapse frame subtree".to_string(),
            ],
            [
                input::CONFIRM.to_string(),
                "Select frame for View transform panel".to_string(),
            ],
        ]
    }

    fn get_name(&self) -> String {
        "TF Tree View".to_string()
    }
}

impl<'a, B: Backend> Drawable<B> for TfTreeView<'a> {
    fn draw(&self, f: &mut Frame<B>) {
        let title_text = vec![Spans::from(Span::styled(
            "TF tree viewer",
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        ))];

        let title = Paragraph::new(title_text)
            .block(Block::default().borders(Borders::ALL))
            .style(Style::default().fg(Color::White))
            .alignment(Alignment::Center)
            .wrap(Wrap { trim: false });

        let areas = Layout::default()
            .direction(Direction::Vertical)
            .horizontal_margin(20)
            .constraints(
                [
                    Constraint::Length(3), // Title + 2 borders
                    Constraint::Length(2),
                    Constraint::Min(1), // Table + header + space
                ]
                .as_ref(),
            )
            .split(f.size());

        let split = Layout::default()
            .direction(Direction::Horizontal)
            .margin(1)
            .constraints([Constraint::Percentage(70), Constraint::Percentage(30)].as_ref())
            .split(areas[2]);

        let items = Tree::new(self.tree.items.clone())
            .block(
                Block::default()
                    .borders(Borders::ALL)
                    .title(self.status_bar.clone()),
            )
            .highlight_style(
                Style::default()
                    .fg(Color::Black)
                    .bg(Color::LightGreen)
                    .add_modifier(Modifier::BOLD),
            )
            .picked_style(
                Style::default()
                    .fg(Color::Black)
                    .bg(Color::Red)
                    .add_modifier(Modifier::BOLD),
            )
            .highlight_symbol(">> ");

        f.render_widget(title, areas[0]);
        let mut state = self.tree.state.clone();
        f.render_stateful_widget(items, split[0], &mut state);

        let p = Paragraph::new(self.tf_echo_details.clone())
            .block(
                Block::default()
                    .title("View transform")
                    .borders(Borders::ALL),
            )
            .wrap(Wrap { trim: true });
        f.render_widget(p, split[1]);
    }
}
