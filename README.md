# dorna2_arm_ros

ROS 2 driver and tool suite for the **Dorna 2 series** robot arms (Dorna TA, Dorna 2, Dorna 2S). Wraps the [`dorna2-python`](https://github.com/dorna-robotics/dorna2-python) WebSocket API and provides a full ROS 2 interface: topics, services, URDF descriptions, Gazebo simulation, and MoveIt 2 motion planning.

## Supported Hardware

| Model | DOF | Notes |
|-------|-----|-------|
| `dorna_ta` | 6 | 6-axis tabletop arm |
| `dorna_2` | 5 | 5-axis arm, shorter base (d1 = 206.4 mm) |
| `dorna_2s` | 5 | 5-axis arm, taller base (d1 = 218.47 mm) |

All models communicate over WebSocket (default `localhost:443`). The robot controller must be reachable on the network before launching the driver.

## Package Overview

```
dorna2_arm_ros/
  dorna2_interfaces/    Custom ROS 2 messages and services (25 srv, 3 msg)
  dorna2_description/   URDF/xacro models, per-link STL meshes, RViz config
  dorna2_driver/        Core driver node (Python, wraps dorna2-python)
  dorna2_bringup/       Unified launch: driver + description + RViz
  dorna2_gazebo/        Gazebo Sim integration with ros2_control
  dorna2_moveit_config/ MoveIt 2 configuration (SRDF, planners, controllers)
```

## Setup (No sudo)

This section covers a complete setup using **conda** (via [robostack](https://robostack.github.io/)), which requires no root privileges. If you already have a system ROS 2 install, skip to [Setup (System ROS 2)](#setup-system-ros-2).

### 1. Install Miniforge (if you don't have conda)

```bash
curl -L -O https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
bash Miniforge3-Linux-x86_64.sh -b -p $HOME/miniforge3
eval "$($HOME/miniforge3/bin/conda shell.bash hook)"
conda init
```

### 2. Create the ROS 2 environment

```bash
conda create -n ros2 -c conda-forge -c robostack-staging \
    ros-humble-desktop \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    colcon-common-extensions \
    python=3.10 \
    compilers cmake pkg-config make

conda activate ros2
```

For simulation (optional):

```bash
conda install -c conda-forge -c robostack-staging \
    ros-humble-ros-gz \
    ros-humble-gz-ros2-control \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-broadcaster
```

For MoveIt (optional):

```bash
conda install -c conda-forge -c robostack-staging \
    ros-humble-moveit
```

### 3. Set up the workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Link or clone this repo
ln -s /path/to/dorna2_arm_ros .

# Install dorna2-python API into the conda env
pip install /path/to/dorna2-python
# -- or --
pip install requests websocket-client numpy
pip install -e /path/to/dorna2-python

# Build
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 4. Test without hardware

```bash
# --- URDF parsing (all 3 models) ---
xacro $(ros2 pkg prefix dorna2_description)/share/dorna2_description/urdf/dorna2.urdf.xacro model:=dorna_ta > /dev/null && echo "dorna_ta URDF: OK"
xacro $(ros2 pkg prefix dorna2_description)/share/dorna2_description/urdf/dorna2.urdf.xacro model:=dorna_2 > /dev/null && echo "dorna_2 URDF: OK"
xacro $(ros2 pkg prefix dorna2_description)/share/dorna2_description/urdf/dorna2.urdf.xacro model:=dorna_2s > /dev/null && echo "dorna_2s URDF: OK"

# --- Visualize in RViz (interactive, no hardware) ---
ros2 launch dorna2_description display.launch.py model:=dorna_ta
# Use joint sliders to verify all 6 joints move correctly.
# Then test 5-DOF:
ros2 launch dorna2_description display.launch.py model:=dorna_2

# --- Verify interfaces built correctly ---
ros2 interface list | grep dorna2
ros2 interface show dorna2_interfaces/srv/JointMove
ros2 interface show dorna2_interfaces/msg/RobotStatus

# --- Launch driver without connecting (dry run) ---
ros2 launch dorna2_driver dorna2_driver.launch.py
# In another terminal:
ros2 node list                    # should show /dorna2_driver
ros2 service list | grep dorna2   # should list 25 services
ros2 topic list | grep dorna2     # should list 4 topics
ros2 param list /dorna2_driver    # should list model, host, port, etc.
ros2 service call /dorna2_driver/cmd/get_info dorna2_interfaces/srv/GetRobotInfo "{}"

# --- Full bringup dry run (driver + RViz, no connection) ---
ros2 launch dorna2_bringup dorna2_bringup.launch.py model:=dorna_ta use_mesh:=false
```

## Setup (System ROS 2)

If you have a system ROS 2 install and sudo access:

```bash
sudo apt install ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-rviz2 \
                 ros-${ROS_DISTRO}-xacro

# Optional: simulation
sudo apt install ros-${ROS_DISTRO}-ros-gz \
                 ros-${ROS_DISTRO}-gz-ros2-control \
                 ros-${ROS_DISTRO}-joint-trajectory-controller \
                 ros-${ROS_DISTRO}-joint-state-broadcaster

# Optional: MoveIt
sudo apt install ros-${ROS_DISTRO}-moveit
```

Then follow steps 3-4 from the no-sudo section above.

## Quick Start

### Visualize the robot (no hardware)

```bash
ros2 launch dorna2_description display.launch.py model:=dorna_ta
```

Launches `robot_state_publisher`, `joint_state_publisher_gui`, and RViz. Drag the joint sliders to move the model.

### Connect to real hardware

```bash
ros2 launch dorna2_bringup dorna2_bringup.launch.py \
    model:=dorna_ta \
    host:=192.168.1.10 \
    auto_connect:=true
```

| Argument | Default | Description |
|----------|---------|-------------|
| `model` | `dorna_ta` | Robot model (`dorna_ta`, `dorna_2`, `dorna_2s`) |
| `host` | `localhost` | Robot controller IP/hostname |
| `port` | `443` | WebSocket port |
| `auto_connect` | `false` | Connect to robot on startup |
| `rviz` | `true` | Launch RViz |
| `use_mesh` | `true` | Use STL meshes (false = primitive shapes) |

### Gazebo simulation

```bash
ros2 launch dorna2_gazebo gazebo.launch.py model:=dorna_ta
```

Spawns the robot in Gazebo with a `joint_trajectory_controller` and `joint_state_broadcaster`. Automatically selects the correct 5-DOF or 6-DOF controller configuration.

### MoveIt 2 motion planning

```bash
ros2 launch dorna2_moveit_config move_group.launch.py model:=dorna_ta
```

Launches the MoveIt `move_group` node with OMPL planning, KDL kinematics, and the appropriate SRDF for the selected model.

## ROS Interface

### Published Topics

All topics are published under the driver node's namespace (`/dorna2_driver/` by default).

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `~/joint_states` | `sensor_msgs/JointState` | 10 Hz | Joint positions in radians |
| `~/cartesian_pose` | `dorna2_interfaces/CartesianPose` | 10 Hz | TCP pose (x, y, z, a, b, c, d, e) in mm/deg |
| `~/robot_status` | `dorna2_interfaces/RobotStatus` | 10 Hz | Connection, alarm, motor, model info |
| `~/io_state` | `dorna2_interfaces/IOState` | 10 Hz | Digital I/O, ADC, PWM state |

### Services

All services are under `~/cmd/`. Each returns `success` (bool) and `message` (string).

**Connection:**
- `~/cmd/connect` -- Connect to robot (host, port)
- `~/cmd/disconnect` -- Disconnect

**Motion:**
- `~/cmd/jmove` -- Joint-space move (joint array, vel, accel, jerk, rel, cont, corner, timeout)
- `~/cmd/lmove` -- Linear Cartesian move (pose array, same params)
- `~/cmd/cmove` -- Circular move (mid/end waypoints, same params)
- `~/cmd/halt` -- Emergency stop current motion
- `~/cmd/jog` -- Incremental jog in joint or Cartesian space

**Homing:**
- `~/cmd/home` -- Home a joint (method: `encoder_index` or `stop`, index, dir, travel, timeout)

**Configuration:**
- `~/cmd/set_joint` -- Set joint position value (for zeroing)
- `~/cmd/set_motor` -- Enable/disable motors
- `~/cmd/set_alarm` -- Set/clear alarm
- `~/cmd/set_tool` -- Set tool frame (rotation matrix + translation)
- `~/cmd/set_toollength` -- Set tool length offset

**I/O:**
- `~/cmd/set_output` -- Set digital output (index, value, queue)
- `~/cmd/set_pwm` -- Configure PWM output (index, enable, freq, duty)
- `~/cmd/get_adc` -- Read ADC values

**Tuning:**
- `~/cmd/set_pid` -- Set PID gains for a joint axis
- `~/cmd/get_pid` -- Read PID gains
- `~/cmd/set_error` -- Set position error threshold/duration
- `~/cmd/set_axis_ratio` -- Set axis gear ratio
- `~/cmd/set_gravity` -- Enable gravity compensation (mass, CoG)

**Safety:**
- `~/cmd/set_emergency` -- Configure emergency stop input
- `~/cmd/probe` -- Probe until input triggers (returns joint positions)

**Diagnostics:**
- `~/cmd/get_info` -- Get firmware version, UID, model, connection state
- `~/cmd/play_script` -- Execute a Dorna script file or raw JSON command

## Driver Parameters

Configured via `dorna2_driver/config/dorna2_params.yaml` or command-line override:

```yaml
dorna2_driver:
  ros__parameters:
    model: "dorna_ta"           # Robot model
    host: "localhost"           # Controller IP
    port: 443                   # WebSocket port
    auto_connect: false         # Connect on startup
    publish_rate: 10.0          # State publishing rate (Hz)
    reconnect_interval: 0.0     # Auto-reconnect period (0 = disabled)
```

Override at launch:

```bash
ros2 launch dorna2_driver dorna2_driver.launch.py \
    --ros-args -p model:=dorna_2 -p host:=10.0.0.14 -p auto_connect:=true
```

## Mesh Files

The `dorna2_description/meshes/` directory contains per-link STL files:

```
meshes/dorna_ta/   base.stl, link0.stl ... link5.stl  (7 files, ~480 MB)
meshes/dorna_2/    base.stl, link0.stl ... link4.stl  (6 files, ~33 MB)
```

The Dorna TA meshes are high-polygon. For better RViz/Gazebo performance, decimate them:

```bash
# MeshLab example: reduce to 10% faces
meshlabserver -i link0.stl -o link0_decimated.stl -s decimate_filter.mlx
```

Set `use_mesh:=false` in any launch file to fall back to primitive cylinder geometry.

If pushing to a remote, consider setting up [git-lfs](https://git-lfs.com/) for the STL files:

```bash
git lfs install
git lfs track "*.stl"
git add .gitattributes
```

## Architecture

```
                    +-----------------+
                    |  dorna2-python  |  (WebSocket API)
                    +--------+--------+
                             |
                    +--------v--------+
                    |  Dorna2Robot    |  Thread-safe wrapper
                    +--------+--------+
                             |
                    +--------v--------+
                    |  Dorna2Node    |  ROS 2 node
                    |  - 4 publishers |
                    |  - 25 services  |
                    +--------+--------+
                             |
              +--------------+--------------+
              |              |              |
         joint_states   cartesian_pose  ~/cmd/*
         (rad, to RSP)  (mm/deg)       (services)
              |
     +--------v--------+
     | robot_state_pub  |----> /tf
     +-----------------+
```

The driver converts between the dorna2 API's native units (degrees, mm) and ROS conventions (radians for `JointState`). Cartesian poses and service parameters remain in the API's native units (mm, degrees) to avoid lossy conversions.

## Extending

**Adding a new service:** Define the `.srv` in `dorna2_interfaces/srv/`, add the method to `Dorna2Robot`, register it in `Dorna2Node._create_services()`, and implement the `_srv_*` callback.

**Custom end-effectors:** Modify the URDF to add links/joints after the `flange` link. Update the SRDF if using MoveIt (add the end-effector group and disable collisions).

**Multiple robots:** Launch multiple driver nodes with different namespaces:

```bash
ros2 launch dorna2_driver dorna2_driver.launch.py \
    --ros-args -r __ns:=/robot1 -p host:=10.0.0.14

ros2 launch dorna2_driver dorna2_driver.launch.py \
    --ros-args -r __ns:=/robot2 -p host:=10.0.0.15
```

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `Connection refused` on connect | Controller not reachable | Verify IP, ping the controller, check firewall |
| Joint states not publishing | Not connected | Call `~/cmd/connect` or set `auto_connect:=true` |
| RViz model invisible | Mesh files missing or too large | Check `meshes/` exists, try `use_mesh:=false` |
| Gazebo crash on spawn | Large mesh files | Decimate STLs (see Mesh Files section) |
| MoveIt "unknown joint" | Wrong model parameter | Ensure `model` matches across all launch files |
| `Invalid model` on startup | Typo in model name | Must be exactly `dorna_ta`, `dorna_2`, or `dorna_2s` |
| `KeyError: dorna_2` | Old dorna2-python config | Driver injects limits automatically; ensure latest dorna2-python |

## License

MIT
