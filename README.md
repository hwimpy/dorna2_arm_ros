# dorna2_arm_ros

ROS 2 driver and tool suite for the **Dorna 2 series** robot arms. Wraps the [`dorna2-python`](https://github.com/dorna-robotics/dorna2-python) WebSocket API and provides a full ROS 2 interface: topics, services, URDF descriptions, Gazebo simulation, and MoveIt 2 motion planning.

| Model | DOF | Status |
|-------|-----|--------|
| `dorna_ta` | 6 | URDF + meshes + driver + Gazebo + MoveIt |
| `dorna_2` | 5 | URDF + meshes + driver + Gazebo + MoveIt |
| `dorna_2s` | 5 | URDF (shares geometry with `dorna_2`, taller base d1 = 218.47 mm) |

All models communicate over WebSocket (default `localhost:443`). The robot controller must be reachable on the network before launching the driver.

## Package Layout

```
dorna2_arm_ros/
├── dorna2_interfaces/       Custom ROS 2 messages (3 msg) and services (25 srv)
├── dorna2_description/      URDF/xacro models, per-link STL meshes, launch, RViz config
│   ├── urdf/
│   │   ├── dorna2.urdf.xacro       Top-level selector (dispatches by model arg)
│   │   ├── dorna_ta.urdf.xacro     Dorna TA: 6-DOF, DH params, mesh visual origins
│   │   ├── dorna_2.urdf.xacro      Dorna 2/2S: 5-DOF
│   │   └── materials.xacro         Shared color definitions
│   ├── meshes/
│   │   ├── dorna_ta/  Active TA meshes
│   │   └── dorna_2/      Dorna 2 meshes (6 STLs, ~11 MB)
│   └── launch/display.launch.py    robot_state_publisher + joint_state_publisher [+ RViz]
├── dorna2_driver/           Core driver node (Python, wraps dorna2-python)
│   ├── dorna2_driver/
│   │   ├── dorna2_node.py           ROS 2 node: publishers, services, parameter handling
│   │   └── dorna2_robot.py          Thread-safe wrapper around the dorna2 WebSocket API
│   └── config/dorna2_params.yaml
├── dorna2_bringup/          Unified launch: driver + description + RViz
├── dorna2_gazebo/           Gazebo Sim integration with ros2_control (5/6-DOF configs)
└── dorna2_moveit_config/    MoveIt 2 configuration (SRDF, OMPL, KDL kinematics)
```

## Setup (Docker -- no sudo required)

Docker is the recommended path for macOS and for any system without root. It gives you a full Ubuntu + ROS 2 Humble environment.

### 1. Start the container

```bash
cd /path/to/dorna-ta-ros     # parent of dorna2_arm_ros/

docker run -it --rm \
    --name dorna2 \
    -v "$(pwd)/dorna2_arm_ros:/ros2_ws/src/dorna2_arm_ros" \
    -p 8765:8765 \
    osrf/ros:humble-desktop-full \
    bash
```

Port 8765 is for Foxglove Bridge (see "Visualize with Foxglove Studio" below).

### 2. Inside the container: install deps and build

```bash
apt-get update && apt-get install -y python3-pip git ros-humble-xacro \
    ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher \
    ros-humble-foxglove-bridge

pip3 install requests websocket-client numpy
pip3 install git+https://github.com/dorna-robotics/dorna2-python.git@master

cd /ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Test without hardware

```bash
# URDF parsing
URDF_DIR=$(ros2 pkg prefix --share dorna2_description)/urdf
xacro $URDF_DIR/dorna2.urdf.xacro model:=dorna_ta  > /dev/null && echo "dorna_ta: OK"
xacro $URDF_DIR/dorna2.urdf.xacro model:=dorna_2   > /dev/null && echo "dorna_2: OK"
xacro $URDF_DIR/dorna2.urdf.xacro model:=dorna_2s  > /dev/null && echo "dorna_2s: OK"

# Verify custom interfaces built
ros2 interface list | grep dorna2
ros2 interface show dorna2_interfaces/srv/JointMove
ros2 interface show dorna2_interfaces/msg/RobotStatus

# Launch driver (dry run, no hardware)
ros2 launch dorna2_driver dorna2_driver.launch.py &
sleep 3
ros2 node list                    # /dorna2_driver
ros2 service list | grep dorna2   # 25 services
ros2 topic list | grep dorna2     # 4 topics
kill %1
```

### 4. Visualize with Foxglove Studio (macOS / Windows / Linux)

[Foxglove Studio](https://foxglove.dev/) connects to the ROS 2 graph over WebSocket. No X11, VNC, or GPU forwarding required -- it runs natively on the host.

**Install Foxglove on the host:**

```bash
brew install --cask foxglove-studio    # macOS
# Or download from https://foxglove.dev/download
```

**Inside the container**, launch the description with the integrated Foxglove bridge:

```bash
source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash

ros2 launch dorna2_description display.launch.py \
    model:=dorna_ta gui:=false use_mesh:=true foxglove:=true
```

To launch with a specific joint pose (radians, comma-separated j0-j5):

```bash
ros2 launch dorna2_description display.launch.py \
    model:=dorna_ta gui:=false use_mesh:=true foxglove:=true \
    pose:='0,0.5236,-0.5236,0,0,0'
```

**Connect from the host:**

1. Open Foxglove Studio.
2. **Open connection** > **Foxglove WebSocket** > `ws://localhost:8765`.
3. Add a **3D** panel.
4. The URDF renders automatically via the `/robot_description` topic.

If the robot doesn't appear, expand **Topics** in the left sidebar and confirm `/robot_description` and `/tf` are being received. If the 3D panel says "no data", click the **Custom layers** section > **+** > choose **URDF** and set the topic to `/robot_description`.

### Optional: Gazebo deps

```bash
apt-get install -y ros-humble-ros-gz ros-humble-gz-ros2-control \
    ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster
```

### Optional: MoveIt deps

```bash
apt-get install -y ros-humble-moveit
```

## Connect to Real Hardware

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
| `use_mesh` | `true` | Use STL meshes (`false` = primitive cylinders) |

## Gazebo Simulation

```bash
ros2 launch dorna2_gazebo gazebo.launch.py model:=dorna_ta
```

Spawns the robot in Gazebo with `joint_trajectory_controller` and `joint_state_broadcaster`. Automatically selects the 5-DOF or 6-DOF controller config.

## MoveIt 2 Motion Planning

```bash
ros2 launch dorna2_moveit_config move_group.launch.py model:=dorna_ta
```

Launches MoveIt `move_group` with OMPL planning, KDL kinematics, and the appropriate SRDF for the selected model.

## ROS Interface

### Published Topics

All under the driver node namespace (`/dorna2_driver/` by default).

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `~/joint_states` | `sensor_msgs/JointState` | 10 Hz | Joint positions (radians) |
| `~/cartesian_pose` | `dorna2_interfaces/CartesianPose` | 10 Hz | TCP pose in mm/deg |
| `~/robot_status` | `dorna2_interfaces/RobotStatus` | 10 Hz | Connection, alarm, motor state |
| `~/io_state` | `dorna2_interfaces/IOState` | 10 Hz | Digital I/O, ADC, PWM |

### Services (25 total, all under `~/cmd/`)

Each returns `success` (bool) + `message` (string).

**Connection:** `connect`, `disconnect`

**Motion:** `jmove` (joint), `lmove` (linear), `cmove` (circular), `halt` (e-stop), `jog`

**Homing:** `home` (encoder_index or stop method)

**Configuration:** `set_joint`, `set_motor`, `set_alarm`, `set_tool`, `set_toollength`

**I/O:** `set_output`, `set_pwm`, `get_adc`

**Tuning:** `set_pid`, `get_pid`, `set_error`, `set_axis_ratio`, `set_gravity`

**Safety:** `set_emergency`, `probe`

**Diagnostics:** `get_info`, `play_script`

### Driver Parameters

Via `dorna2_driver/config/dorna2_params.yaml`:

```yaml
dorna2_driver:
  ros__parameters:
    model: "dorna_ta"
    host: "localhost"
    port: 443
    auto_connect: false
    publish_rate: 10.0
    reconnect_interval: 0.0    # seconds, 0 = disabled
```

## Architecture

### Data Flow

```
                    +-----------------+
                    |  dorna2-python  |  WebSocket API (JSON over WS)
                    +--------+--------+
                             |
                    +--------v--------+
                    |  Dorna2Robot    |  Thread-safe Python wrapper
                    |  (deg/mm API)  |  (dorna2_robot.py)
                    +--------+--------+
                             |
                    +--------v--------+
                    |  Dorna2Node    |  ROS 2 lifecycle node
                    |  4 publishers   |  (dorna2_node.py)
                    |  25 services    |
                    +--------+--------+
                             |
              +--------------+--------------+
              |              |              |
         joint_states   cartesian_pose  ~/cmd/*
         (radians)      (mm/deg)       (services)
              |
     +--------v--------+
     | robot_state_pub  |----> /tf, /tf_static
     +-----------------+
              |
     +--------v--------+
     | foxglove_bridge  |----> WebSocket :8765
     +-----------------+
              |
     +--------v--------+
     | Foxglove Studio  |  (native host app)
     +-----------------+
```

The driver converts between the dorna2 API's native units (degrees, mm) and ROS conventions (radians for `JointState`). Cartesian poses and service parameters remain in API-native units (mm, degrees) to avoid lossy conversions.

### URDF / Mesh Pipeline

The Dorna TA URDF (`dorna_ta.urdf.xacro`) defines a 6-DOF kinematic chain using DH parameters extracted from `dorna2-python`:

```
world ─[fixed]─> base_link ─[j0: Z rot]─> link0 ─[j1: Z rot, RotX(90°)]─> link1
  ─[j2: Z rot]─> link2 ─[j3: Z rot, RotX(90°)+RotZ(90°)]─> link3
  ─[j4: Z rot, RotX(90°)+RotZ(180°)]─> link4
  ─[j5: Z rot, RotX(90°)+RotZ(180°)]─> link5 ─[fixed]─> flange ─[fixed]─> tcp
```

| Parameter | Value (m) | Description |
|-----------|-----------|-------------|
| `d0` | 0.230018 | Base height (world to joint0) |
| `a1` | 0.080 | Link0 length (joint0 to joint1) |
| `a2` | 0.210 | Link1 length (joint1 to joint2) |
| `d3` | 0.0418 | Wrist offset (joint2 to joint3) |
| `d4` | 0.175 | Wrist link (joint3 to joint4) |
| `d5` | -0.089 | Wrist link (joint4 to joint5, negative = reversed) |
| `d6` | 0.035 | Flange offset (joint5 to TCP) |

**Mesh coordinate mapping:** The STL meshes were exported from the Dorna TA STEP assembly (`Dorna_TA.step`) and are stored in the **CAD global frame**. The URDF visual origins apply a per-link rigid transform to map from CAD frame to each link's local frame:

- **Rotation:** RotZ(-90 deg) -- CAD Y axis maps to URDF X (arm extension direction), CAD X maps to -URDF Y, CAD Z maps to URDF Z (vertical).
- **Translation:** Offset by the cumulative FK position to place each mesh at its link origin.

Currently, the full arm body (assemblies A2-A5, covering link1 through the wrist) is merged into `link1.stl` to avoid visual disconnection at articulated poses. This is a consequence of the CAD STL exports containing large housing shells that span the joint2 boundary as single connected meshes -- any split produces visible displacement when the joint rotates. See "Known Limitations" below.

Set `use_mesh:=false` in any launch to fall back to primitive cylinder geometry.

## Mesh Files

```
meshes/dorna_ta_v6/   base.stl, link0.stl, link1.stl, link2-5.stl (placeholders)   ~12 MB
meshes/dorna_ta/      Original per-link decimated STLs + .stl.orig backups           ~19 MB
meshes/dorna_2/       base.stl .. link4.stl                                          ~11 MB
```

The active mesh set for `dorna_ta` is `dorna_ta_v6`. Link1 contains the full arm geometry (A2-A5 assemblies merged and decimated to 143k faces). Links 2-5 are placeholders -- see "Known Limitations" below.

The `dorna_ta/` directory retains the original per-link decimated STLs and their `.stl.orig` high-poly backups for future re-export work. The `.stl.orig` files are excluded by `.gitignore`.

For git hosting, use [git-lfs](https://git-lfs.com/) for STL files:

```bash
git lfs install
git lfs track "*.stl"
git add .gitattributes
```

## Known Limitations

**Joint2-5 mesh visualization:** The Dorna TA arm geometry is currently merged into link1, so only joint0 (base rotation) and joint1 (shoulder) produce visible mesh changes in Foxglove/RViz. Joints 2-5 are kinematically correct in the TF tree and will affect downstream frames (link2-link5, flange, tcp), but the mesh does not visually articulate at these joints.

This is caused by the original CAD STL files containing large housing shells that span the joint2 boundary as single connected meshes. Splitting these shells at the joint produces a 140mm cross-section that visibly displaces when the joint rotates. The fix requires per-link CAD re-export from the original Dorna TA assembly, where each link body is exported as a separate mesh in its link-local coordinate frame.

**Dorna 2/2S meshes:** The `dorna_2` mesh set has per-link STLs but has not been validated with the same rigor as `dorna_ta`. Visual origins may need adjustment.

## Extending

**Adding a service:** Define `.srv` in `dorna2_interfaces/srv/`, implement in `Dorna2Robot`, register in `Dorna2Node._create_services()`.

**Custom end-effectors:** Add links/joints after `flange` in the URDF. Update the SRDF if using MoveIt.

**Multiple robots:**

```bash
ros2 launch dorna2_driver dorna2_driver.launch.py \
    --ros-args -r __ns:=/robot1 -p host:=10.0.0.14

ros2 launch dorna2_driver dorna2_driver.launch.py \
    --ros-args -r __ns:=/robot2 -p host:=10.0.0.15
```

## License

MIT
