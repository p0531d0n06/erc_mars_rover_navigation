# erc_mars_rover_navigation

This is the ROS 2 repository for creating a self-driving Mars rover for the European Rover Challenge (ERC).

## Overview

The rover features autonomous navigation capabilities using LiDAR-based SLAM (Simultaneous Localization and Mapping) combined with multi-camera ArUco marker detection for waypoint navigation. The system leverages Nav2 for path planning and obstacle avoidance, with visual servoing for precise docking maneuvers.

**Key Features:**
- SLAM-based localization (RTAB-Map or Cartographer) with LiDAR point clouds
- Nav2 stack with Hybrid A\* planner and MPPI controller
- Multi-camera ArUco detection with GPU acceleration on Jetson
- PnP pose estimation fused with LiDAR depth
- Visual servoing for precise final docking
- Full TF2 transform tree for coordinate frame management

## Hardware Setup

- **Compute Platform**: NVIDIA Jetson Orin Nano Super
- **Operating System**: NVIDIA Ubuntu 22.04 LTS (JetPack)
- **LiDAR Sensor**: Unitree 4D LiDAR L2
- **Cameras**: Four RGB cameras (front/left/right/back)
- **Robotic Arm**: Custom arm with camera mount for visual search
- **ROS Version**: ROS 2 Humble

## Repository Structure

```
erc_mars_rover_navigation/
├── point_lio_ros2/       # Point-LIO SLAM package for LiDAR odometry
├── ref_models/           # 3D reference models (STL files)
│   ├── rover.stl         # Rover chassis 3D model
│   └── arm.stl           # Robotic arm 3D model
├── Important Assets/     # Additional project resources
└── README.md
```

## Reference Models

The `ref_models/` directory contains 3D STL models used for visualization, simulation, and hardware reference:

| Model | Description |
|-------|-------------|
| `rover.stl` | Complete rover chassis and body structure |
| `arm.stl` | Robotic arm assembly with camera mount |

These models can be used with RViz2 for visualization or imported into simulation environments like Gazebo.

## Point-LIO Integration

This project uses [Point-LIO ROS2](point_lio_ros2/) for robust LiDAR-inertial odometry. Point-LIO provides:
- High-bandwidth odometry under aggressive motions
- Real-time mapping capabilities
- Support for Unitree L2 LiDAR with built-in IMU

## High-Level Navigation Algorithm

The navigation system uses a five-phase architecture that leverages SLAM, Nav2, and visual servoing for robust autonomous operation.

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                         PHASE 0: SYSTEM INITIALIZATION                      │
│  • Launch lifecycle nodes (LiDAR, cameras, SLAM, Nav2, ArUco detectors)    │
│  • Establish TF2 tree: map → odom → base_link → sensors                    │
│  • Load calibration, costmaps, and ArUco waypoint queue                    │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│              PHASE 1: CONCURRENT SLAM + MULTI-CAMERA DETECTION              │
│  • Nav2 frontier exploration / waypoint follower                           │
│  • SLAM ingests LiDAR, maintains occupancy grid, performs loop closure     │
│  • 4 ArUco detectors publish to /aruco/detections concurrently             │
│  • Arm pan/tilt adjusts dynamically for ground-plane coverage              │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                          Detection event received
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│            PHASE 2: ID VALIDATION & POSE ESTIMATION                         │
│  • Compare marker ID against front of waypoint queue                       │
│  • If mismatch → discard, return to Phase 1                                │
│  • If match → cancel Nav2 goal, begin pose estimation:                     │
│    - solvePnP (IPPE/EPnP) for camera-frame pose                           │
│    - Project to LiDAR frustum, extract median depth                        │
│    - Fuse PnP + LiDAR depth (weighted average)                            │
│    - Transform via TF2: camera_link → base_link → odom → map              │
│    - Register as SLAM landmark                                             │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│            PHASE 3: DOCK POSE COMPUTATION & NAV2 NAVIGATION                 │
│  • Compute dock pose: offset along marker normal, heading = yaw + π        │
│  • Send Nav2 NavigateToPose action goal                                    │
│  • Hybrid A* planner + MPPI controller + LiDAR costmap avoidance          │
│  • On arrival (< 0.3m): switch to visual servoing                         │
│    - Minimize pixel error between detected/target corners                  │
│    - Generate /cmd_vel until error < 5px, heading < 2°                    │
│  • Verify: detection + pixel error + LiDAR range ± 2cm                    │
│  • On failure: retry N times or approach from lateral offset              │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                    PHASE 4: QUEUE MANAGEMENT & LOOP                         │
│  • Remove marker ID from queue, log timestamp + dock pose                  │
│  • Snapshot SLAM map to disk                                               │
│  • If queue non-empty → re-activate Nav2 exploration → Phase 1            │
│  • If queue empty → Phase 5                                                │
└─────────────────────────────────────────────────────────────────────────────┘
                                      │
                                      ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│              PHASE 5: RETURN TO ORIGIN & GRACEFUL SHUTDOWN                  │
│  • Nav2 NavigateToPose to origin with full obstacle avoidance             │
│  • Visual servo dock to origin ArUco marker                               │
│  • Deactivate Nav2, save final SLAM map, shutdown lifecycle nodes         │
└─────────────────────────────────────────────────────────────────────────────┘
```

### Phase 0 — System Initialization

Launch the ROS2 lifecycle manager and activate all lifecycle nodes in order:
1. Unitree LiDAR L2 driver
2. Four RGB camera drivers
3. SLAM node (RTAB-Map or Cartographer ingesting LiDAR point clouds)
4. Nav2 stack (costmap servers, Smac Hybrid A\* planner, MPPI controller, BT navigator, behavior server)
5. Four ArUco detector nodes (one per camera, each pinned to a separate Jetson CPU core with OpenCV CUDA or TensorRT)

**TF2 Transform Tree:**
```
map → odom → base_link → lidar_link
                       → camera_front_link
                       → camera_left_link
                       → camera_right_link
                       → camera_back_link
                       → arm kinematic chain
```

**Initialization Tasks:**
- Load camera intrinsic matrices and distortion coefficients
- Load camera–LiDAR extrinsic calibration
- Configure Nav2 costmaps (global + local) to consume SLAM occupancy grid and LiDAR scan
- Record current SLAM pose as origin landmark
- Load ordered ArUco waypoint ID queue from parameter file
- Publish initial pose estimate to SLAM and Nav2's AMCL/SLAM localizer

### Phase 1 — Concurrent SLAM Mapping + Multi-Camera ArUco Detection

Activate Nav2 frontier exploration (or waypoint follower if environment is partially known) for autonomous traversal.

**SLAM Operations:**
- Continuously ingest LiDAR point clouds
- Maintain occupancy grid
- Publish map → odom → base_link TF chain
- Perform loop closure when revisiting geometry

**ArUco Detection:**
- All four ArUco detector nodes publish to `/aruco/detections` independently
- Arm orientation adjusted dynamically via pan/tilt ROS2 node to maximize ground-plane coverage
- If no detection arrives, rover continues exploring

### Phase 2 — Detection Event, ID Validation & Pose Estimation

On any detection event:
1. Read marker ID and compare against front of waypoint queue
2. If ID does not match → discard and resume Phase 1
3. If ID matches → cancel active Nav2 goal, begin pose estimation

**Pose Estimation Pipeline:**
```
┌─────────────────────┐     ┌─────────────────────┐
│   solvePnP          │     │   LiDAR Projection  │
│   (IPPE/EPnP)       │     │   Frustum Extraction│
│   ↓                 │     │   Median Depth      │
│   Camera-frame pose │     │   ↓                 │
└─────────┬───────────┘     └─────────┬───────────┘
          │                           │
          └─────────┬─────────────────┘
                    ▼
          ┌─────────────────────┐
          │  Weighted Fusion    │
          │  (LiDAR weighted    │
          │   higher)           │
          └─────────┬───────────┘
                    ▼
          ┌─────────────────────┐
          │  TF2 Transform      │
          │  camera → base →    │
          │  odom → map         │
          └─────────┬───────────┘
                    ▼
          ┌─────────────────────┐
          │  Register as SLAM   │
          │  landmark           │
          └─────────────────────┘
```

### Phase 3 — Dock Pose Computation & Nav2 Navigation

**Dock Pose Calculation:**
- Position: marker's map position offset by configured docking distance along marker's normal vector
- Heading: yaw = marker_yaw + π (facing the marker)

**Navigation:**
- Send as Nav2 `NavigateToPose` action goal
- Hybrid A\* global planner generates kinematically-feasible path
- MPPI local controller tracks path
- LiDAR-backed costmap handles dynamic obstacle avoidance
- Built-in behavior tree runs recovery actions (clear costmap, spin, backup) on recoverable failures

**Visual Servoing (within 0.3m threshold):**
```
WHILE pixel_error > 5px OR heading_error > 2°:
    error = target_corners - detected_corners
    cmd_vel = proportional_controller(error)
    publish(/cmd_vel)
```

**Docking Verification:**
- Confirm marker detection
- Pixel error within tolerance
- LiDAR range matches expected docking distance (±2cm)
- On failure: retry visual servoing up to N times
- On persistent failure: Nav2 approach from lateral offset angle

### Phase 4 — Queue Management & Loop

1. Remove front marker ID from queue
2. Log timestamp and final dock pose
3. Snapshot SLAM map to disk
4. If queue non-empty → re-activate Nav2 exploration → return to Phase 1
5. If queue empty → proceed to Phase 5

### Phase 5 — Return to Origin & Graceful Shutdown

1. Retrieve origin pose from SLAM map
2. Send Nav2 `NavigateToPose` goal with full obstacle avoidance
3. Perform final visual-servo dock to origin ArUco marker
4. Deactivate Nav2 and exploration nodes via lifecycle manager
5. Save final SLAM map to disk
6. Shut down all remaining lifecycle nodes cleanly

---

## Hardware-Specific Implementation Notes

### LiDAR L2 and SLAM

RTAB-Map with LiDAR as primary odometry source (ICP odometry) is well-tested on ROS2 and handles the Unitree L2's point cloud natively. For lighter footprint, Cartographer with a 2D lidar slice from L2 is an alternative.

**Key Point:** Keep the SLAM graph-optimizer running throughout — loop closures are the main reason to prefer this over pure odometry.

### Arm Kinematics in TF2

If the arm has joints that move (not just a fixed bracket), you need:
- `joint_state_publisher` node
- URDF describing the arm

This allows TF2 to compute the live camera-to-base_link transform as the arm moves. **Without this, PnP estimates will be wrong whenever the arm angle changes.**

### GPU Pipeline (Jetson Orin Nano)

| Approach | Description |
|----------|-------------|
| **OpenCV CUDA** | `cv::aruco::detectMarkers` on GPU with minimal code changes |
| **TensorRT** | Custom pipeline with lightweight detector (e.g., YOLOv8-based ArUco or corner detector) — pushes detection latency well below 10ms per frame |

**Thread Management:** Pin each camera's ROS2 node to a separate CPU core via `taskset` or ROS2 executor configuration to avoid thread contention.

### Docking Verification

Rather than only checking pixel error, add a secondary LiDAR check: after visual servo completes, verify the point-cloud range to the marker plane is within ±2cm of target docking distance. This catches cases where camera angle makes pixel error look correct but physical distance is wrong.

---

## Legacy: Traversability-Based RRT* Path Planning

> **Note:** The current implementation uses Nav2's Smac Hybrid A\* planner. The RRT\* algorithm below is retained for reference and potential future use in extremely rough terrain scenarios.

<details>
<summary>Click to expand legacy RRT* documentation</summary>

The rover can alternatively use a **Traversability-Based RRT\*** algorithm for path planning in rough terrain. This implementation is based on:

> Takemura, R. and Ishigami, G. "Traversability-Based RRT* for Planetary Rover Path Planning in Rough Terrain with LIDAR Point Cloud Data." *Journal of Robotics and Mechatronics*, Vol.29 No.5, 2017. [DOI: 10.20965/jrm.2017.p0838](https://doi.org/10.20965/jrm.2017.p0838)

### Why RRT* Over Grid-Based Methods?

Traditional grid-based methods (like GESTALT) have limitations:
- Computation increases exponentially with grid resolution
- Discretized maps fail to represent complex terrain features
- Path quality varies based on grid resolution/shape

RRT* works directly on LiDAR point cloud data (continuous space) and asymptotically converges to optimal paths.

### Algorithm Overview

**Rover Configuration (Node):**
```
q_i = (x_i, y_i, z_i, φ_i, θ_i, ψ_i)

where:
  (x, y, z)  = Center of gravity coordinates
  φ (phi)   = Roll angle
  θ (theta) = Pitch angle  
  ψ (psi)   = Yaw angle
```

**Cost Function:**
```
C(q_i) = W_L × (L_i / N_L) + W_φ × (φ_i / N_φ) + W_θ × (θ_i / N_θ) + W_ψ × (Δψ_i / N_ψ)
```

**Default Parameters:**

| Parameter | Value | Description |
|-----------|-------|-------------|
| R_max | 0.830 m | Maximum sampling radius |
| R_min | 0.415 m | Minimum sampling radius |
| φ_th | 10.0° | Roll angle threshold |
| θ_th | 10.0° | Pitch angle threshold |
| ψ_th | 50.0° | Heading angle threshold |
| δ_th | 10.2° | Elevation angle threshold |

</details>

## Installation

### Prerequisites

1. ROS 2 Humble installed on Ubuntu 22.04
2. Required ROS packages:
   ```bash
   # Core packages
   sudo apt-get install ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-visualization-msgs
   
   # Nav2 stack
   sudo apt-get install ros-humble-navigation2 ros-humble-nav2-bringup
   
   # SLAM packages (choose one)
   sudo apt-get install ros-humble-rtabmap-ros  # RTAB-Map
   # OR
   sudo apt-get install ros-humble-cartographer ros-humble-cartographer-ros  # Cartographer
   
   # ArUco detection
   sudo apt-get install ros-humble-aruco-opencv
   
   # TF2 and robot state
   sudo apt-get install ros-humble-tf2-ros ros-humble-robot-state-publisher ros-humble-joint-state-publisher
   ```
3. Eigen3:
   ```bash
   sudo apt-get install libeigen3-dev
   ```
4. OpenCV with CUDA (for Jetson GPU acceleration):
   ```bash
   # OpenCV is pre-installed on JetPack; verify CUDA support:
   python3 -c "import cv2; print(cv2.getBuildInformation())" | grep CUDA
   ```
5. Unitree LiDAR SDK2 (for L2 LiDAR):
   ```bash
   git clone https://github.com/unitreerobotics/unilidar_sdk2.git
   cd unilidar_sdk2/unitree_lidar_ros2
   colcon build
   ```

### Building

```bash
cd ~/ros2_ws/src
git clone <this-repository>
cd ..
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Launch Point-LIO with Unitree L2 LiDAR

```bash
source install/setup.bash
ros2 launch point_lio mapping_unilidar_l2.launch.py
```

### Visualize Reference Models in RViz2

Load the STL files from `ref_models/` as robot models or markers for visualization.

## License

See [LICENSE](LICENSE) file for details.