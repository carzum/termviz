# TermViz - ROS visualization on the terminal

TermViz is a visualizer for the Robot Operating System (ROS) framework, that runs completely on the terminal.
This allows the visualizer to run directly on the robot (e.g., via ssh) without any configuration on an external computer.
It can also be useful in case the robot has a firewall or in situations in which no graphics server can be run.

## How to use

To launch the visualizer, just run the `termviz` executable.

The program looks for a configuration file named `termviz.yaml` in `~/.config/termviz/` first, then in `/etc/termviz/`. If the file is not found, it will prompt to create a default one in `~/.config/termviz/termviz.yaml`. Alternative it is possible to pass a configuration file directly to the executable: `termviz <myconfig>.yaml`.

The program requires a running ROS master and an available TF between the robot frame (`base_link` by default) and a static frame (`map` by default).

Once the visualizer starts, every configured topic, as well as the robot footprint (obtained from the ROS parameter `/footprint`) and frame, should be visible in the viewport. Pressing `h` shows the help screen, which will describe the current mode and the keymap relative to the current mode. The mode can be switched using the number keys and the help screen will update accordingly.

### Send pose mode

The mode allows to publish a `geometry_msgs::PoseWithCovarianceStamped` message on a topic. The desired pose can be selected by moving the outline of the robot in the map. Confirming the operation (`Enter` by default) sends a PoseStamped message on the topic specified under `send_pose_topic` in the configuration file.

### Teleoperate mode

The mode allows to teleoperate the robot by sending `geometry_msgs::Twist` messages on the specified topic (`cmd_vel` by default). The messages are continuosly sent. Any unmapped key switches the sent messages to 0, i.e., stops the robot.
Settings can be found under `teleop` in the configuration file.

### Image mode

This node allows to visualize images received on the topics specified under `image_topics` in the configuration file.

## Supported message types

- geometry_msgs::PoseArray
- geometry_msgs::PoseStamped
- nav_msgs::OccupancyGrid
- nav_msgs::Path
- sensor_msgs::Image
- sensor_msgs::LaserScan
- sensor_msgs::PointCloud2
- visualization_msgs::Marker
- visualization_msgs::MarkerArray

## Default config

Here is the commented default config file:
```yaml
---
fixed_frame: map                # Fixed frame.
robot_frame: base_link          # Fobot frame.
map_topics:                     # nav_msgs::OccupancyGrid topics.
  - topic: map                  # Topic name.
    color:                      # Color of the occupied cells.
      r: 255
      g: 255
      b: 255
    threshold: 1                # Threshold value to consider a cell occupied (cell containing a lower value are not visualized).
laser_topics:                   # sensor_msgs::LaserScan topics.
  - topic: scan                 # Topic name.
    color:                      # Color of the laser data.
      r: 200
      g: 0
      b: 0
marker_topics:                  # visualization_msgs::Marker topics.
  - topic: marker               # Topic name.
image_topics:                   # sensor_msgs::Image topics.
  - topic: image_rect           # Topic name.
    rotation: 0                 # Default rotation in degrees. Supported angles: 0, 90, 180, 270.
marker_array_topics:            # visualization_msgs::MarkerArray topics.
  - topic: marker_array         # Topic name.
path_topics:                    # nav_msgs::Path topics.
  - topic: path                 # Topic name.
    style: line                 # Visualization style. Supported: arrow, axis, line.
    color:                      # Color of the arrow or line.
      r: 0
      g: 255
      b: 0
    length: 0.2                 # Length of the arrow or axes.
pointcloud2_topics:             # sensor_msgs::PointCloud2 topics.
  - topic: pointcloud2          # Topic name.
    use_rgb: false              # If true, the points are colorized according to their RGB values. If false, they are colorized according to their height, i.e., their z coordinate in the static frame.
pose_array_topics:              # geometry_msgs::PoseArray.
  - topic: pose_array           # Topic name.
    style: arrow                # Visualization style. Supported: arrow, axis.
    color:                      # Color of the arrow.
      r: 255
      g: 0
      b: 0
    length: 0.2                 # Length of the axes.
pose_stamped_topics:            # geometry_msgs::PoseStamped topics.
  - topic: pose_stamped         # The topic name.
    style: axis                 # Visualization style. Supported: arrow, axis.
    color:                      # Color of the arrow.
      r: 255
      g: 0
      b: 0
    length: 0.2                 # Length of the axes.
send_pose_topic: initialpose    # Topic on which to publish poses in Send Pose mode.
target_framerate: 30            # Refresh rate of the visualization
axis_length: 0.5                # Length of the axes of the robot frame
visible_area:                   # Default boundaries of the visible areas. Determines the initial level of zoom.
  - -5.0
  - 5.0
  - -5.0
  - 5.0
zoom_factor: 0.1                # Step for increasing/decreasing the zoom.
key_mapping:                    # Keymap
  Cancel: Esc
  Zoom in: "="
  Increment step: k
  Decrement step: j
  Clockwise rotation: e
  Right: d
  Down: s
  Up: w
  Left: a
  Switch to mode 2: t
  Confirm: Enter
  Switch to mode 3: i
  Show help: h
  Zoom out: "-"
  Counter-clockwise rotation: q
teleop:                        # Parameters for the Teleoperate mode.
  default_increment: 0.1       # Default velocity increment when pressing a key.
  increment_step: 0.1          # Step for increasing the velocity increment.
  cmd_vel_topic: cmd_vel       # Topic on which to publish the velocity commands.
```

## Maintainers

- Carsten Zumsande (termviz@zumsande.eu)
- Emanuele Palazzolo (emanuele.palazzolo@gmail.com)
- Michael März (michael0maerz@gmail.com)