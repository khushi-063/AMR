# AMR — Autonomous Mobile Robot

A ROS 2 differential-drive robot with LiDAR, IMU, and camera sensors running in Ignition Gazebo. Supports real-time SLAM mapping, Nav2-based autonomous navigation with AMCL localisation on a pre-built map, and multiple simulated indoor environments.

---

## Table of Contents

- [About](#about)
- [System Architecture](#system-architecture)
- [Robot Hardware](#robot-hardware)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [ROS 2 Topics](#ros-2-topics)
- [Configuration](#configuration)
- [Progress](#progress)
- [Common Issues](#common-issues)

---

## About

Custom differential-drive AMR built for indoor navigation research. The full simulation stack covers the complete sensor-to-navigation pipeline:

- **Ignition Gazebo** physics simulation with multiple indoor worlds (empty, office, warehouse, testing)
- **SLAM Toolbox** for real-time occupancy-grid mapping with loop closure *(mapping mode)*
- **AMCL** particle-filter localisation on a pre-saved map *(navigation mode)*
- **Nav2** for global path planning (A*), local trajectory following (DWB), and recovery behaviours
- **ros_gz_bridge** for full sensor, odometry, and TF bridging between Gazebo and ROS 2

---

## System Architecture

```
Ignition Gazebo
  │
  ├─ DiffDrive plugin   ──→  /model/my_robot/odometry  ──→  [gz bridge]  ──→  /odom
  ├─ LiDAR sensor       ──→  /scan                     ──→  [gz bridge]  ──→  /scan
  ├─ IMU sensor         ──→  /imu/data                 ──→  [gz bridge]  ──→  /imu/data
  ├─ Camera             ──→  /camera/image_raw          ──→  [gz bridge]  ──→  /camera/image_raw
  ├─ JointState         ──→  /world/.../joint_state     ──→  [gz bridge]  ──→  /joint_states
  └─ TF                 ──→  /model/my_robot/tf         ──→  [gz bridge]  ──→  /tf

[robot_state_publisher]  ←─ robot_description (URDF/Xacro)
    └─ TF: base_link → lidar, imu_link, camera_link, wheels, casters

[amcl]  ←─ /scan + /odom + pre-built map                    ← navigation mode
    └─ TF: map → odom
    └─ localises the robot within the known map

[slam_toolbox]  ←─ /scan + TF(odom→base_link)               ← mapping mode (swap with amcl)
    └─ TF: map → odom
    └─ publishes: /map (OccupancyGrid, updated every 3 s)

[nav2 stack]  ←─ /map + /scan + /odom + TF tree
    ├─ map_server       — serves the pre-built occupancy map
    ├─ planner_server   — NavFn A* global planner
    ├─ controller_server — DWB local planner (Dynamic Window Approach)
    ├─ behavior_server  — spin / backup / wait recovery behaviours
    └─ bt_navigator     — navigate_to_pose action server → /cmd_vel
```

---

## Robot Hardware

| Component | Specification |
|---|---|
| Base | Custom differential-drive chassis |
| Drive wheels | 2 × motorised (wheel_1, wheel_2), ~230 mm separation |
| Casters | 2 × passive ball casters |
| LiDAR | 360° LiDAR, range 0.3–12 m, 20 Hz |
| IMU | 6-DOF IMU at 50 Hz |
| Camera | RGB camera, `/camera/image_raw` |
| Base mass | ~21 kg |

---

## Project Structure

```
AMR/
├── amr_description/               # Robot model package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── launch/
│   │   └── display.launch.xml     # View robot in RViz2 only (no sim)
│   ├── meshes/
│   │   ├── base_link.stl
│   │   ├── wheel_1.stl
│   │   ├── wheel_2.stl
│   │   ├── ball_caster_wheel_1.stl
│   │   ├── ball_caster_wheel__1__1.stl
│   │   └── lidar.stl
│   ├── rviz/
│   │   └── amr.rviz               # RViz2 configuration
│   └── urdf/
│       ├── robot.urdf.xacro                        # Top-level URDF entry point
│       ├── slam_lidar_midplate_middle.xacro         # Links, joints, inertials
│       ├── slam_lidar_midplate_middle.gazebo        # Gazebo sensor plugins
│       ├── slam_lidar_midplate_middle.ros2control   # ros2_control interface
│       └── materials.xacro                         # Material definitions
│
├── gazebo_bringup/                # Simulation bringup package
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── config/
│   │   ├── gazebo_bridge.yaml     # ros_gz_bridge topic mappings
│   │   └── slam_toolbox_params.yaml
│   ├── launch/
│   │   └── amr.launch.xml         # Main launch file (Gazebo + Nav2 + RViz2)
│   └── worlds/
│       ├── empty_sensors.sdf      # Minimal world for sensor testing
│       ├── office.sdf             # Office environment
│       ├── testing.sdf            # General-purpose test world
│       └── warehouse.sdf          # Warehouse environment
│
└── nav2_bringup/                  # Navigation stack package
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    │   └── nav2.launch.xml        # Nav2 stack (included by amr.launch.xml)
    ├── maps/
    │   ├── my_map.pgm             # Pre-built occupancy map image
    │   └── my_map.yaml            # Map metadata
    └── params/
        └── nav2_params.yaml       # AMCL, planner, controller, costmap params
```

---

## Prerequisites

| Dependency | Version | Install |
|---|---|---|
| Ubuntu | 22.04 LTS | — |
| ROS 2 | Humble | [docs.ros.org](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) |
| Ignition Gazebo | Fortress | [gazebosim.org](https://gazebosim.org/docs/fortress/install_ubuntu) |
| SLAM Toolbox | Humble | `sudo apt install ros-humble-slam-toolbox` |
| Nav2 | Humble | `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup` |
| ros_gz | Humble-Fortress | `sudo apt install ros-humble-ros-gz` |
| xacro | Humble | `sudo apt install ros-humble-xacro` |

---

## Installation

```bash
# 1. Create workspace
mkdir -p ~/slam/src && cd ~/slam/src

# 2. Clone
git clone https://github.com/khushi-063/AMR.git .

# 3. Install ROS dependencies
cd ~/slam
rosdep install --from-paths src --ignore-src -r -y

# 4. Build
colcon build

# 5. Source
source install/setup.bash
# Add permanently:
echo "source ~/slam/install/setup.bash" >> ~/.bashrc
```

---

## Usage

### Navigation with pre-built map (default)

Launches Gazebo, the robot, ros_gz_bridge, Nav2 (AMCL + map server), and RViz2:

```bash
ros2 launch gazebo_bringup amr.launch.xml
```

Choose a world:

```bash
ros2 launch gazebo_bringup amr.launch.xml world:=warehouse
ros2 launch gazebo_bringup amr.launch.xml world:=office
ros2 launch gazebo_bringup amr.launch.xml world:=testing
ros2 launch gazebo_bringup amr.launch.xml world:=empty_sensors
```

Once launched, set the initial pose in RViz2 using **2D Pose Estimate**, then send navigation goals with **2D Goal Pose**.

---

### SLAM mapping mode

To build a new map instead of navigating on a saved one, edit `amr.launch.xml`:

1. **Uncomment** the `slam_toolbox` node block
2. **Comment out** the `<include ... nav2.launch.xml/>` line

Then launch and drive the robot manually to map the environment:

```bash
ros2 launch gazebo_bringup amr.launch.xml
```

Save the finished map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/slam/src/nav2_bringup/maps/my_map
```

---

### View robot model only

```bash
ros2 launch amr_description display.launch.xml
```

---

## ROS 2 Topics

### Subscribed

| Topic | Type | Source |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller |

### Published

| Topic | Type | Publisher |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo bridge |
| `/odom` | `nav_msgs/Odometry` | Gazebo bridge (DiffDrive plugin) |
| `/imu/data` | `sensor_msgs/Imu` | Gazebo bridge |
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo bridge |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | Gazebo bridge |
| `/joint_states` | `sensor_msgs/JointState` | Gazebo bridge |
| `/map` | `nav_msgs/OccupancyGrid` | map_server / SLAM Toolbox |
| `/tf` / `/tf_static` | `tf2_msgs/TFMessage` | Gazebo bridge + RSP + AMCL |

### TF Tree

```
map
 └─ odom                     (AMCL or SLAM Toolbox)
     └─ base_link             (Gazebo bridge /tf)
         ├─ lidar
         ├─ imu_link
         ├─ camera_link
         │   └─ camera_link_optical
         ├─ wheel_1
         ├─ wheel_2
         ├─ ball_caster_wheel_1
         └─ ball_caster_wheel__1__1
```

---

## Configuration

### SLAM Toolbox (`gazebo_bringup/config/slam_toolbox_params.yaml`)

| Parameter | Value | Notes |
|---|---|---|
| `resolution` | 0.05 m | Map cell size |
| `max_laser_range` | 12.0 m | Matches LiDAR range |
| `map_update_interval` | 3.0 s | Occupancy grid publish rate |
| `minimum_travel_distance` | 0.5 m | Distance between scan-match updates |
| `do_loop_closing` | true | Pose-graph loop closure for drift correction |

### AMCL (`nav2_bringup/params/nav2_params.yaml`)

| Parameter | Value | Notes |
|---|---|---|
| `min_particles` | 1000 | Lower bound on particle count |
| `max_particles` | 3000 | Upper bound on particle count |
| `update_min_d` | 0.25 m | Min translation before AMCL updates |
| `update_min_a` | 0.215 rad | Min rotation before AMCL updates |
| `robot_model_type` | DifferentialMotionModel | Matches diff-drive kinematics |

### Nav2 (`nav2_bringup/params/nav2_params.yaml`)

| Parameter | Value | Notes |
|---|---|---|
| Planner | NavFn A* | `allow_unknown: true` |
| Controller | DWB | Dynamic Window Approach local planner |
| Max linear velocity | 0.26 m/s | `max_vel_x` |
| Max angular velocity | 1.0 rad/s | `max_vel_theta` |
| Local costmap | 3 × 3 m rolling window | `robot_radius: 0.17 m` |
| Local inflation radius | 0.55 m | Safety clearance |
| Global inflation radius | 0.65 m | Routes planned away from walls |
| Goal tolerance (xy) | 0.25 m | |
| Recovery behaviours | spin, backup, wait | |

---

## Progress

### Done
- [x] Differential-drive robot URDF with accurate inertials, collision meshes, and joints
- [x] LiDAR, IMU, RGB camera sensors in Ignition Gazebo simulation
- [x] Full ros_gz_bridge configuration (sensors, clock, cmd_vel, odom, TF, joint states)
- [x] SLAM Toolbox online async mapping with loop closure
- [x] Nav2 stack — global planner (A*), local controller (DWB), costmaps, recovery behaviours
- [x] AMCL localisation on pre-built map
- [x] Multiple indoor simulation worlds (empty, office, warehouse, testing)
- [x] RViz2 configuration for navigation visualisation

### Planned
- [ ] EKF odometry fusion (wheel odometry + IMU) for improved localisation
- [ ] Frontier-based autonomous map exploration
- [ ] Hardware deployment on the physical robot

---

## Common Issues

**`lifecycle_manager` fails to bring up nodes**
→ A BT plugin listed in `nav2_params.yaml` may not be installed. Check the `[bt_navigator]` error and remove the missing plugin name from `plugin_lib_names`.

**`map → odom` TF not available**
→ AMCL has not received an initial pose estimate. Use **2D Pose Estimate** in RViz2 to set the robot's starting location on the map.

**`src refspec main does not match any` on git push**
→ No commits exist yet. Run `git add`, `git commit`, then push.

**Robot not moving after Nav2 goal**
→ Check that `/cmd_vel` is bridged correctly in `gazebo_bridge.yaml` and that the lifecycle manager reports all nodes active (`ros2 lifecycle get /bt_navigator`).

**RViz2 shows no map**
→ Confirm `my_map.yaml` exists at `nav2_bringup/maps/` and that `map_server` is in the `active` lifecycle state.
