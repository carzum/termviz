use crate::app_modes::{input, AppMode, BaseMode, Drawable};
use crate::config::Color as ConfigColor;
use crate::config::TermvizConfig;
use crate::config::{ImageListenerConfig, ListenerConfig, ListenerConfigColor, PoseListenerConfig};
use crate::ros;
use rand::Rng;
use tui::backend::Backend;
use tui::layout::{Alignment, Constraint, Direction, Layout};
use tui::style::{Color, Modifier, Style};
use tui::text::{Span, Spans};
use tui::widgets::{Block, Borders, List, ListItem, ListState, Paragraph, Wrap};
use tui::Frame;

fn topic_manager_description() -> Vec<String> {
    vec!["Topic manager can enable and disable displayed topics".to_string()]
}

fn topic_manager_keymap() -> Vec<[String; 2]> {
    vec![
        [
            input::UP.to_string(),
            "Selects the previous item in the active list".to_string(),
        ],
        [
            input::DOWN.to_string(),
            "Selects the next item in the active list".to_string(),
        ],
        [
            input::RIGHT.to_string(),
            "Shifts an element to the right if the supported topic list is active".to_string(),
        ],
        [
            input::LEFT.to_string(),
            "Shifts an element to the left if the active list is active".to_string(),
        ],
        [
            input::ROTATE_RIGHT.to_string(),
            "Changes the list where items are selected to the active topics list".to_string(),
        ],
        [
            input::ROTATE_LEFT.to_string(),
            "Changes the list where items are selected to the supported topics list".to_string(),
        ],
        [input::CONFIRM.to_string(), "Saves to config".to_string()],
    ]
}

pub struct LazyTopicManager {
    config: Option<TermvizConfig>,
    manager: Option<TopicManager>,
}

impl LazyTopicManager {
    pub fn new(config: TermvizConfig) -> LazyTopicManager {
        LazyTopicManager {
            config: Some(config),
            manager: None,
        }
    }

    fn ensure_initialized(&mut self) -> &mut TopicManager {
        if self.manager.is_none() {
            let config = self
                .config
                .take()
                .expect("topic manager config missing during initialization");
            self.manager = Some(TopicManager::new(config));
        }
        self.manager.as_mut().unwrap()
    }
}

#[derive(Clone)]
struct SelectableTopics {
    // `items` is the state managed by your application.
    items: Vec<[String; 2]>,
    // `state` is the state that can be modified by the UI. It stores the index of the selected
    // item as well as the offset computed during the previous draw call (used to implement
    // natural scrolling).
    state: ListState,
}

impl SelectableTopics {
    fn new(items: Vec<[String; 2]>) -> SelectableTopics {
        SelectableTopics {
            items,
            state: ListState::default(),
        }
    }

    // Select the next item. This will not be reflected until the widget is drawn in the
    // `Terminal::draw` callback using `Frame::render_stateful_widget`.
    pub fn next(&mut self) {
        let i = match self.state.selected() {
            Some(i) => {
                if i >= self.items.len() - 1 {
                    0
                } else {
                    i + 1
                }
            }
            None => 0,
        };
        self.state.select(Some(i));
    }

    // Select the previous item. This will not be reflected until the widget is drawn in the
    // `Terminal::draw` callback using `Frame::render_stateful_widget`.
    pub fn previous(&mut self) {
        let i = match self.state.selected() {
            Some(i) => {
                if i == 0 {
                    self.items.len() - 1
                } else {
                    i - 1
                }
            }
            None => 0,
        };
        self.state.select(Some(i));
    }

    pub fn add(&mut self, element: [String; 2]) {
        self.items.push(element);
    }

    // Default to 0 if none is selected, the handling of empty vectors should be
    // handled by the caller
    pub fn pop(&mut self) -> [String; 2] {
        let i = match self.state.selected() {
            Some(i) => {
                if i > self.items.len() - 1 {
                    self.items.len() - 1
                } else {
                    i
                }
            }
            None => 0,
        };
        self.items.remove(i)
    }
}

pub struct TopicManager {
    // Topic Manger loads the active and supported topics into two lists.
    // The User can shift elements between available and selected topics.
    // topics can only be present in on of the lists.
    availible_topics: SelectableTopics,
    selected_topics: SelectableTopics,
    config: TermvizConfig,
    selection_mode: bool,
    was_saved: bool,
}

fn topic_type_pair(topic: &str, topic_type: &str) -> [String; 2] {
    [topic.to_string(), topic_type.to_string()]
}

fn active_topics_from_config(config: &TermvizConfig) -> Vec<[String; 2]> {
    let active_laser_topics: Vec<[String; 2]> = config
        .laser_topics
        .iter()
        .map(|i| topic_type_pair(&i.topic, "sensor_msgs/LaserScan"))
        .collect();
    let active_marker_array_topics: Vec<[String; 2]> = config
        .marker_array_topics
        .iter()
        .map(|i| topic_type_pair(&i.topic, "visualization_msgs/MarkerArray"))
        .collect();
    let active_marker_topics: Vec<[String; 2]> = config
        .marker_topics
        .iter()
        .map(|i| topic_type_pair(&i.topic, "visualization_msgs/Marker"))
        .collect();
    let active_pose_stamped_topics: Vec<[String; 2]> = config
        .pose_stamped_topics
        .iter()
        .map(|i| topic_type_pair(&i.topic, "geometry_msgs/PoseStamped"))
        .collect();
    let active_pose_array_topics: Vec<[String; 2]> = config
        .pose_array_topics
        .iter()
        .map(|i| topic_type_pair(&i.topic, "geometry_msgs/PoseArray"))
        .collect();
    let active_path_topics: Vec<[String; 2]> = config
        .path_topics
        .iter()
        .map(|i| topic_type_pair(&i.topic, "nav_msgs/Path"))
        .collect();
    let active_image_topics: Vec<[String; 2]> = config
        .image_topics
        .iter()
        .map(|i| topic_type_pair(&i.topic, "sensor_msgs/Image"))
        .collect();
    let polygon_stamped_topics: Vec<[String; 2]> = config
        .polygon_stamped_topics
        .iter()
        .map(|i| topic_type_pair(&i.topic, "geometry_msgs/PolygonStamped"))
        .collect();

    [
        active_image_topics,
        active_laser_topics,
        active_marker_array_topics,
        active_marker_topics,
        active_path_topics,
        active_pose_array_topics,
        active_pose_stamped_topics,
        polygon_stamped_topics,
    ]
    .concat()
}

fn supported_topic_types() -> Vec<String> {
    vec![
        "geometry_msgs/PoseArray".to_string(),
        "geometry_msgs/PoseStamped".to_string(),
        "nav_msgs/Path".to_string(),
        "sensor_msgs/Image".to_string(),
        "sensor_msgs/LaserScan".to_string(),
        "visualization_msgs/Marker".to_string(),
        "visualization_msgs/MarkerArray".to_string(),
        "geometry_msgs/PolygonStamped".to_string(),
    ]
}

fn rebuild_config_from_selected_topics(
    config: &TermvizConfig,
    selected_topics: &[[String; 2]],
) -> TermvizConfig {
    let mut config = config.clone();
    config.laser_topics.clear();
    config.marker_array_topics.clear();
    config.marker_topics.clear();
    config.pose_stamped_topics.clear();
    config.pose_array_topics.clear();
    config.path_topics.clear();
    config.image_topics.clear();
    config.polygon_stamped_topics.clear();

    let mut rng = rand::thread_rng();
    for topic in selected_topics {
        match topic[1].as_str() {
            "sensor_msgs/LaserScan" => config.laser_topics.push(ListenerConfigColor {
                topic: topic[0].clone(),
                color: ConfigColor {
                    r: rng.gen_range(0..255),
                    g: rng.gen_range(0..255),
                    b: rng.gen_range(0..255),
                },
            }),
            "visualization_msgs/MarkerArray" => config.marker_array_topics.push(ListenerConfig {
                topic: topic[0].clone(),
            }),
            "visualization_msgs/Marker" => config.marker_topics.push(ListenerConfig {
                topic: topic[0].clone(),
            }),
            "geometry_msgs/PoseStamped" => config.pose_stamped_topics.push(PoseListenerConfig {
                topic: topic[0].clone(),
                color: ConfigColor {
                    r: rng.gen_range(0..255),
                    g: rng.gen_range(0..255),
                    b: rng.gen_range(0..255),
                },
                length: 0.2,
                style: "axis".to_string(),
            }),
            "geometry_msgs/PoseArray" => config.pose_array_topics.push(PoseListenerConfig {
                topic: topic[0].clone(),
                color: ConfigColor {
                    r: rng.gen_range(0..255),
                    g: rng.gen_range(0..255),
                    b: rng.gen_range(0..255),
                },
                length: 0.2,
                style: "axis".to_string(),
            }),
            "nav_msgs/Path" => config.path_topics.push(PoseListenerConfig {
                topic: topic[0].clone(),
                color: ConfigColor {
                    r: rng.gen_range(0..255),
                    g: rng.gen_range(0..255),
                    b: rng.gen_range(0..255),
                },
                length: 0.2,
                style: "axis".to_string(),
            }),
            "sensor_msgs/Image" => config.image_topics.push(ImageListenerConfig {
                topic: topic[0].clone(),
                rotation: 0,
            }),
            "geometry_msgs/PolygonStamped" => {
                config.polygon_stamped_topics.push(ListenerConfigColor {
                    topic: topic[0].clone(),
                    color: ConfigColor {
                        r: rng.gen_range(0..255),
                        g: rng.gen_range(0..255),
                        b: rng.gen_range(0..255),
                    },
                })
            }
            _ => (),
        }
    }

    config
}

impl TopicManager {
    pub fn new(config: TermvizConfig) -> TopicManager {
        let config = config.clone();
        let all_active_topics = active_topics_from_config(&config);
        let supported_topic_types = supported_topic_types();
        // Collect all topics, which:
        //  - are supported
        //  - are inactive
        let mut supported_topics: Vec<[String; 2]> = ros::topics()
            .into_iter()
            .map(|(name, datatype)| [name, datatype])
            .filter(|el| supported_topic_types.contains(&el[1].to_string()))
            .filter(|el| !all_active_topics.contains(&el))
            .collect();
        supported_topics.sort();

        let mut supported_topic_list = SelectableTopics::new(supported_topics);
        supported_topic_list.state.select(Some(0));

        // Fill the state manager with active and supported topics
        TopicManager {
            availible_topics: supported_topic_list,
            selected_topics: SelectableTopics::new(all_active_topics),
            config: config,
            selection_mode: true,
            was_saved: false,
        }
    }

    pub fn shift_active_element_right(&mut self) {
        if self.availible_topics.items.is_empty() {
            return;
        }
        let x = self.availible_topics.pop();
        self.selected_topics.add(x);
    }
    pub fn shift_active_element_left(&mut self) {
        if self.selected_topics.items.is_empty() {
            return;
        }
        let x = self.selected_topics.pop();
        self.availible_topics.add(x);
    }

    pub fn save(&mut self) {
        let config = rebuild_config_from_selected_topics(&self.config, &self.selected_topics.items);

        // Store and exit termviz
        let _ = confy::store("termviz", "termviz", &(config));
        self.was_saved = true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn config_with_image_topic() -> TermvizConfig {
        let mut config = TermvizConfig::default();
        config.image_topics = vec![ImageListenerConfig {
            topic: "camera/image".to_string(),
            rotation: 270,
        }];
        config.path_topics = vec![PoseListenerConfig {
            topic: "robot/path".to_string(),
            style: "line".to_string(),
            color: ConfigColor { r: 0, g: 255, b: 0 },
            length: 0.2,
        }];
        config
    }

    #[test]
    fn topic_manager_uses_image_topics_for_active_list() {
        let active_topics = active_topics_from_config(&config_with_image_topic());

        assert!(active_topics.contains(&topic_type_pair("camera/image", "sensor_msgs/Image")));
        assert!(!active_topics.contains(&topic_type_pair("robot/path", "sensor_msgs/Image")));
    }

    #[test]
    fn topic_manager_save_keeps_selected_image_topics() {
        let mut config = TermvizConfig::default();
        config.image_topics = vec![ImageListenerConfig {
            topic: "stale/image".to_string(),
            rotation: 90,
        }];
        let config = rebuild_config_from_selected_topics(
            &config,
            &[topic_type_pair("camera/image", "sensor_msgs/Image")],
        );

        assert_eq!(config.image_topics.len(), 1);
        assert_eq!(config.image_topics[0].topic, "camera/image");
        assert_eq!(config.image_topics[0].rotation, 0);
    }
}

impl<B: Backend> BaseMode<B> for TopicManager {}

impl AppMode for TopicManager {
    fn run(&mut self) {}
    fn reset(&mut self) {}
    fn get_description(&self) -> Vec<String> {
        topic_manager_description()
    }

    fn handle_input(&mut self, input: &String) {
        if self.selection_mode {
            match input.as_str() {
                input::UP => self.availible_topics.previous(),
                input::DOWN => self.availible_topics.next(),
                input::RIGHT => self.shift_active_element_right(),
                input::ROTATE_RIGHT => {
                    self.selection_mode = false;
                    self.selected_topics.state.select(Some(0));
                    self.availible_topics.state.select(None);
                }
                input::CONFIRM => self.save(),
                _ => (),
            }
        } else {
            match input.as_str() {
                input::UP => self.selected_topics.previous(),
                input::DOWN => self.selected_topics.next(),
                input::LEFT => self.shift_active_element_left(),
                input::ROTATE_LEFT => {
                    self.selection_mode = true;
                    self.availible_topics.state.select(Some(0));
                    self.selected_topics.state.select(None);
                }
                input::CONFIRM => self.save(),
                _ => (),
            }
        }
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        topic_manager_keymap()
    }

    fn get_name(&self) -> String {
        "Topic Manager".to_string()
    }
}

impl AppMode for LazyTopicManager {
    fn run(&mut self) {
        if let Some(manager) = self.manager.as_mut() {
            manager.run();
        }
    }

    fn reset(&mut self) {
        self.ensure_initialized().reset();
    }

    fn handle_input(&mut self, input: &String) {
        self.ensure_initialized().handle_input(input);
    }

    fn get_description(&self) -> Vec<String> {
        self.manager
            .as_ref()
            .map(|manager| manager.get_description())
            .unwrap_or_else(topic_manager_description)
    }

    fn get_keymap(&self) -> Vec<[String; 2]> {
        self.manager
            .as_ref()
            .map(|manager| manager.get_keymap())
            .unwrap_or_else(topic_manager_keymap)
    }

    fn get_name(&self) -> String {
        self.manager
            .as_ref()
            .map(|manager| manager.get_name())
            .unwrap_or_else(|| "Topic Manager".to_string())
    }
}

impl<B: Backend> Drawable<B> for LazyTopicManager {
    fn draw(&self, f: &mut Frame<B>) {
        if let Some(manager) = self.manager.as_ref() {
            manager.draw(f);
        }
    }
}

impl<B: Backend> BaseMode<B> for LazyTopicManager {}

impl<B: Backend> Drawable<B> for TopicManager {
    fn draw(&self, f: &mut Frame<B>) {
        let title_text = vec![Spans::from(Span::styled(
            "Topic Manager",
            Style::default().fg(Color::Red).add_modifier(Modifier::BOLD),
        ))];
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
        let title = Paragraph::new(title_text)
            .block(Block::default().borders(Borders::ALL))
            .style(Style::default().fg(Color::White))
            .alignment(Alignment::Center)
            .wrap(Wrap { trim: false });

        if !self.was_saved {
            let left_chunks = Layout::default()
                .direction(Direction::Horizontal)
                .margin(1)
                .constraints([Constraint::Percentage(50), Constraint::Percentage(50)].as_ref())
                .split(areas[2]);
            // Widget creation
            let items: Vec<ListItem> = self
                .availible_topics
                .items
                .iter()
                .map(|i| ListItem::new(format!("{} : {}", i[0], i[1])))
                .collect();
            // The `List` widget is then built with those items.
            let list = List::new(items)
                .highlight_style(Style::default().add_modifier(Modifier::BOLD))
                .block(
                    Block::default()
                        .title("Available Topics")
                        .borders(Borders::ALL),
                )
                .highlight_symbol(">> ");

            let selected_items: Vec<ListItem> = self
                .selected_topics
                .items
                .iter()
                .map(|i| ListItem::new(i[0].as_ref()))
                .collect();
            // The `List` widget is then built with those items.
            let selected_list = List::new(selected_items)
                .highlight_style(Style::default().add_modifier(Modifier::BOLD))
                .block(
                    Block::default()
                        .title("Active Topics")
                        .borders(Borders::ALL),
                )
                .highlight_symbol(">> ");
            // Finally the widget is rendered using the associated state. `events.state` is
            // effectively the only thing that we will "remember" from this draw call.
            f.render_widget(title, areas[0]);
            f.render_stateful_widget(
                list,
                left_chunks[0],
                &mut self.availible_topics.state.clone(),
            );
            f.render_stateful_widget(
                selected_list,
                left_chunks[1],
                &mut self.selected_topics.state.clone(),
            );
        } else {
            let user_info = Paragraph::new(Spans::from(Span::raw(
                "Config has been saved, restart termviz to use it. \n Switch to any other mode to continue"
            )))
            .block(Block::default().borders(Borders::NONE))
            .style(Style::default().fg(Color::White))
            .alignment(Alignment::Center)
            .wrap(Wrap { trim: false });
            f.render_widget(user_info, areas[1]);
        }
    }
}
