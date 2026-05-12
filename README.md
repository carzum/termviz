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

### ROS1 / ROS2 feature flags

TermViz does not enable a ROS backend by default. You must select exactly one backend feature: `ros1` or `ros2`.

- ROS1 build (explicit):
  ```bash
  cargo build --release --features ros1
  ```
- ROS2 build:
  - Ensure you have a ROS2 environment sourced (e.g. `source /opt/ros/<distro>/setup.bash`) so `ROS_DISTRO` is a ROS2 distro (not `noetic`).
  - Then build with:
    ```bash
    cargo build --release --features ros2
    ```

Repo-local Cargo aliases are also available:

- ROS1: `cargo build-ros1`, `cargo run-ros1`, `cargo test-ros1`
- ROS2: `cargo build-ros2`, `cargo run-ros2`, `cargo test-ros2`

Note: for some ROS1 setups you may need `ROSRUST_MSG_PATH=/usr/share/` when building.

After the build succeeded, the executable will be located in `target/release/` and can be used directly. No external libraries are needed, so it can be copied directly on a robot or another computer.

## How to use

To launch the visualizer, just run the `termviz` executable.


The program looks for a configuration file named `termviz.yml` in `~/.config/termviz/` first, then in `/etc/termviz/`. If no config is found, it starts with sensible defaults. If an existing config is missing newer fields (from a newer TermViz version), those fields are filled from defaults so older configs keep working. When no custom config path is provided, TermViz will persist the (possibly updated) config to `~/.config/termviz/termviz.yml` so future runs use the same settings. You may also pass a configuration file directly: `termviz <myconfig>.yml`.

The program requires a running ROS master and an available TF between the robot frame (`base_link` by default) and a static frame (`map` by default). If the ROS parameter `/footprint` is present, its polygon will be used to show the robot footprint.

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
```

## Maintainers

- Carsten Zumsande (termviz@zumsande.eu)
- Emanuele Palazzolo (emanuele.palazzolo@gmail.com)
- Michael März (michael0maerz@gmail.com)


## Integration tests

This repository includes black-box integration tests under `tests/`.

Requirements for ROS-gated tests
- `roscore` and `rosrun` must be available in `PATH` (ROS1/Noetic or compatible).
- Tests will create short-lived ROS processes and temporary log directories.

Running the tests

```bash
cargo test --features ros1
```

Run only the CLI/help test:

```bash
cargo test --features ros1 --test cli_help
```

Enable and run ROS-gated tests (local ROS install required):

```bash
TERMVIZ_IT_ROS=1 cargo test --features ros1
```

CI notes
- To run ROS-gated tests in CI, use a runner image that contains ROS (Noetic) and ensure `roscore`/`rosrun` are in `PATH`. Set `TERMVIZ_IT_ROS=1` in the job environment. Add `--test-threads=1` if tests interfere with each other in your CI environment.
