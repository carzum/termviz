use crate::config::{ListenerConfigColor, PoseListenerConfig, ListenerConfig, ImageListenerConfig};
use crate::config::Color as ConfigColor;
use crate::app_modes::{input, AppMode, BaseMode, Drawable};
use tui::backend::Backend;
use tui::layout::{Alignment, Constraint, Direction, Layout};
use crate::config::TermvizConfig;
use tui::style::{Color, Modifier, Style};
use tui::text::{Span, Spans};
use tui::Frame;
use tui::widgets::{Block, Borders, Paragraph, ListState, Wrap, ListItem, List};
use rand::Rng;


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
    pub fn pop(&mut self) -> [String; 2]{
        let i = match self.state.selected() {
            Some(i) => {
                if i > self.items.len() - 1 {
                    self.items.len() - 1
                }
                else {
                    i
                }
            },
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

impl TopicManager {
    pub fn new(config: TermvizConfig) ->  TopicManager
    {
        let config = config.clone();

        // Get all topics currently active in the config and sort them by topic type
        let active_laser_topics: Vec<[String; 2]> = config.laser_topics.iter().map(
            |i| {[i.topic.clone(),  "sensor_msgs/LaserScan".to_string()]}
        ).collect();
        let active_marker_array_topics: Vec<[String; 2]> = config.marker_array_topics.iter().map(
            |i| {[i.topic.clone(),  "visualization_msgs/MarkerArray".to_string()]}
        ).collect();
        let active_marker_topics: Vec<[String; 2]> = config.marker_topics.iter().map(
            |i| {[i.topic.clone(),  "visualization_msgs/Marker".to_string()]}
        ).collect();
        let active_pose_stamped_topics: Vec<[String; 2]> = config.pose_stamped_topics.iter().map(
            |i| {[i.topic.clone(),  "geometry_msgs/PoseStamped".to_string()]}
        ).collect();
        let active_pose_array_topics: Vec<[String; 2]> = config.pose_array_topics.iter().map(
            |i| {[i.topic.clone(),  "geometry_msgs/PoseArray".to_string()]}
        ).collect();
        let active_path_topics: Vec<[String; 2]> = config.path_topics.iter().map(
            |i| {[i.topic.clone(),  "nav_msgs/Path".to_string()]}
        ).collect();
        let active_image_topics: Vec<[String; 2]> = config.path_topics.iter().map(
            |i| {[i.topic.clone(),  "sensor_msgs/Image".to_string()]}
        ).collect();
        // Collect them into a big list
        let all_active_topics = [
            active_image_topics,
            active_laser_topics,
            active_marker_array_topics,
            active_marker_topics,
            active_path_topics,
            active_pose_array_topics,
            active_pose_stamped_topics,
        ].concat();

        
        // We could get this from config, but would need some breaking changes in config
        let supported_topic_types = vec![
            "geometry_msgs/PoseArray".to_string(),
            "geometry_msgs/PoseStamped".to_string(),
            "nav_msgs/Path".to_string(),
            "sensor_msgs/Image".to_string(),
            "sensor_msgs/LaserScan".to_string(),
            "visualization_msgs/Marker".to_string(),
            "visualization_msgs/MarkerArray".to_string(),
        ];
        // Collect all topics, which:
        //  - are supported
        //  - are inactive
        let supported_topics: Vec<[String; 2]>= rosrust::topics()
            .unwrap()
            .iter()
            .map(|topic|{
                [topic.name.to_string(),
                 topic.datatype.to_string()
                ]
            })
            .filter(|el| supported_topic_types.contains(&el[1].to_string()))
            .filter(|el| !all_active_topics.contains(&el))
            .collect();

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
        if self.availible_topics.items.is_empty() {return}
        let x = self.availible_topics.pop();
        self.selected_topics.add(x);
    }
    pub fn shift_active_element_left(&mut self) {
        if self.selected_topics.items.is_empty() {return}
        let x = self.selected_topics.pop();
        self.availible_topics.add(x);
    }

    pub fn save(&mut self) {
        let mut config = self.config.clone();

        // Flush all to get a new config
        config.laser_topics.clear();
        config.marker_array_topics.clear();
        config.marker_topics.clear();
        config.pose_stamped_topics.clear();
        config.pose_array_topics.clear();
        config.path_topics.clear();

        // Fill the respective topics
        // The current implementation hardcodes where the topics must go
        // This could be handled by a more descriptive config structure
        let mut rng = rand::thread_rng();
        for topic in self.selected_topics.items.iter() {
            match topic[1].clone().as_ref() {
                "sensor_msgs/LaserScan" => config.laser_topics.push(
                    ListenerConfigColor{
                        topic: topic[0].clone(),
                        color: ConfigColor{r:rng.gen_range(0..255), g:rng.gen_range(0..255), b:rng.gen_range(0..255)},
                    }
                ),
                "visualization_msgs/MarkerArray" => config.marker_array_topics.push(
                    ListenerConfig {
                        topic: topic[0].clone(),
                    }
                ),
                "visualization_msgs/Marker" => config.marker_topics.push(
                    ListenerConfig {
                        topic: topic[0].clone(),
                    }
                ),
                "geometry_msgs/PoseStamped" => config.pose_stamped_topics.push(
                    PoseListenerConfig{
                        topic: topic[0].clone(),
                        color: ConfigColor{r:rng.gen_range(0..255), g:rng.gen_range(0..255), b:rng.gen_range(0..255)},
                        length: 0.2,
                        style: "axis".to_string(),
                    }
                ),
                "geometry_msgs/PoseArray" => config.pose_array_topics.push(
                    PoseListenerConfig{
                        topic: topic[0].clone(),
                        color: ConfigColor{r:rng.gen_range(0..255), g:rng.gen_range(0..255), b:rng.gen_range(0..255)},
                        length: 0.2,
                        style: "axis".to_string(),
                    }
                ),
                "nav_msgs/Path" => config.path_topics.push(
                    PoseListenerConfig{
                        topic: topic[0].clone(),
                        color: ConfigColor{r:rng.gen_range(0..255), g:rng.gen_range(0..255), b:rng.gen_range(0..255)},
                        length: 0.2,
                        style: "axis".to_string(),
                    }
                ),
                "sensor_msg/Image" => config.image_topics.push(
                    ImageListenerConfig{
                        topic: topic[0].clone(),
                        rotation: 0,
                    }
                ),

                _ => (),
            }
        }

        // Store and exit termviz
        let _ = confy::store("termviz", "termviz", &(config));
        self.was_saved = true
    }

}

impl<B: Backend> BaseMode<B> for TopicManager {}

impl AppMode for TopicManager {
    fn run(&mut self) {}
    fn reset(&mut self) {}
    fn get_description(&self) -> Vec<String> {
        vec!["Topic manager can enable and disable displayed topics".to_string()]
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
        }
        else {
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
            [
                input::CONFIRM.to_string(),
                "Saves to config".to_string(),
            ],
        ]
    }

    fn get_name(&self) -> String {
        "Topic Manager".to_string()
    }
}

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

        if ! self.was_saved 
        {
            let left_chunks = Layout::default()
                .direction(Direction::Horizontal)
                .margin(1)
                .constraints(
                    [
                        Constraint::Percentage(50),
                        Constraint::Percentage(50),
                    ]
                    .as_ref(),
                )
                .split(areas[2]);
            // Widget creation
            let items: Vec<ListItem>= self.availible_topics.items.iter().map(|i| ListItem::new(
                    format!("{} : {}", i[0], i[1])
                )
            ).collect();
            // The `List` widget is then built with those items.
            let list = List::new(items)
                .highlight_style(Style::default().add_modifier(Modifier::BOLD))
                .block( Block::default().title("Available Topics").borders(Borders::ALL))
                .highlight_symbol(">> ");

            let selected_items: Vec<ListItem>= self.selected_topics.items.iter().map(|i| ListItem::new(i[0].as_ref())).collect();
            // The `List` widget is then built with those items.
            let selected_list = List::new(selected_items)
                .highlight_style(Style::default().add_modifier(Modifier::BOLD))
                .block( Block::default().title("Active Topics").borders(Borders::ALL))
                .highlight_symbol(">> ");
            // Finally the widget is rendered using the associated state. `events.state` is
            // effectively the only thing that we will "remember" from this draw call.
            f.render_widget(title, areas[0]);
            f.render_stateful_widget(list, left_chunks[0], &mut self.availible_topics.state.clone());
            f.render_stateful_widget(selected_list, left_chunks[1], &mut self.selected_topics.state.clone());
        }
        else {
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