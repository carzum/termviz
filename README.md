# TermViz - ROS visualization on the terminal

![Animation](images/usage.gif)

TermViz is a visualizer for the Robot Operating System (ROS) framework, that runs completely on the terminal.
This allows the visualizer to run directly on the robot (e.g., via ssh) without any configuration on an external computer. Additionally, it is lightweight and really fast to start and to quit.
It can also be useful in case the robot has a firewall or in situations in which no graphics server can be run.
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

## Installation

Get the source code:
```bash
git clone git@github.com:carzum/termviz.git
```

Build the project via cargo:
```bash
cargo build --release
```

After the build succeeded, the executable will be located in `target/release/` and can be used directly. No external libraries are needed, so it can be copied directly on a robot or another computer.

## How to use

To launch the visualizer, just run the `termviz` executable.

The program looks for a configuration file named `termviz.yml` in `~/.config/termviz/` first, then in `/etc/termviz/`. If the file is not found, it prompts the user to create a default one. Alternatively, it is possible to pass a configuration file directly to the executable: `termviz <myconfig>.yml`.

The program requires a running ROS master and an available TF between the robot frame (`base_link` by default) and a static frame (`map` by default). If the ROS parameter `/footprint`, it will be used to show the footprint of the robot.

Pressing `h` shows the help screen, which will describe the current mode and the keymap relative to the current mode. The mode can be switched using the number keys and the help screen will update accordingly.

### Send pose mode

The mode allows to publish a pose message on a topic, for example to send an initial pose estimate to a localization system or a goal pose for the navigation stack. The supported types are `geometry_msgs::Pose`, `geometry_msgs::PoseStamped`, and `geometry_msgs::PoseWithCovarianceStamped`. The desired pose can be selected by moving the outline of the robot in the map. Confirming the operation (`Enter` by default) publishes the pose on the selected topic among those specified under `send_pose_topics` in the configuration file. The target topic can be selected using the "next" and "previous" keys (`n` and `b` by default).

### Teleoperate mode

The mode allows to teleoperate the robot by sending `geometry_msgs::Twist` messages on the specified topic (`cmd_vel` by default). The messages are continuously sent. Any unmapped key switches the sent messages to 0, i.e., stops the robot.
Settings can be found under `teleop` in the configuration file.

If the parameter 'publish_cmd_vel_when_idle' is set to true (default), the mode will keep publishing STOP (all velocities 0).
Otherwise the command is only sent once! This should allow users to teleoperate robots without blocking them actively

### Image mode

This mode allows to visualize images received on the topics specified under `image_topics` in the configuration file.

### Topic Manager

The topic manager can add and remove topics int the termviz config. When confirmed the config will be stored and termviz must be restarted.
Only supported topics are displayed, topics can only be in the active or in the available list.

### Tf Tree View

Similar to rqt_tf_tree, it displays the TF tree and allows users to interactively echo any two arbitrary frames.

## Default config

Here is the commented default config file:
```yaml
---
fixed_frame: map                # Fixed frame.
robot_frame: base_link          # Robot frame.
map_topics:                     # nav_msgs::OccupancyGrid topics.
  - topic: map                  # Topic name.
    color:                      # Color of the occupied cells.
      r: 255
      g: 255
      b: 255
    threshold: 1                # Threshold value to consider a cell occupied (cells containing a lower value are not visualized).
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
send_pose_topics:               # Topics on which to publish poses in Send Pose mode.
  - topic: pose                 # The topic name.
    msg_type: PoseStamped       # The topic's type. Supported are Pose, PoseStamped and PoseWithCovarianceStamped.
target_framerate: 30            # Refresh rate of the visualization. Lower this if the ssh connection is slow.
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
  publish_cmd_vel_when_idle: true # If true keep publishing 0 velocities, only publish once otherwise
tf_frames_service_name: /tf_service/tf2_frames
```

## Maintainers

- Carsten Zumsande (termviz@zumsande.eu)
- Emanuele Palazzolo (emanuele.palazzolo@gmail.com)
- Michael März (michael0maerz@gmail.com)
