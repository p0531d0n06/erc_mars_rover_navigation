# erc_mars_rover_navigation

This is the ROS 2 repository for creating a self-driving Mars rover for the European Rover Challenge (ERC).

## Overview

The rover features autonomous navigation capabilities using LiDAR-based SLAM (Simultaneous Localization and Mapping) combined with ArUco marker detection for waypoint navigation. The system uses a robotic arm with an attached camera for waypoint detection and visual search.

## Hardware Setup

- **Compute Platform**: NVIDIA Jetson Orin Nano Super
- **Operating System**: NVIDIA Ubuntu 22.04 LTS (JetPack)
- **LiDAR Sensor**: Unitree 4D LiDAR L2
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

## Traversability-Based RRT* Path Planning

The rover uses a **Traversability-Based RRT\*** (Rapidly-Exploring Random Trees Star) algorithm for path planning in rough terrain. This implementation is based on the research paper:

> Takemura, R. and Ishigami, G. "Traversability-Based RRT* for Planetary Rover Path Planning in Rough Terrain with LIDAR Point Cloud Data." *Journal of Robotics and Mechatronics*, Vol.29 No.5, 2017. [DOI: 10.20965/jrm.2017.p0838](https://doi.org/10.20965/jrm.2017.p0838)

### Why RRT* Over Grid-Based Methods?

Traditional grid-based methods (like GESTALT) have limitations:
- Computation increases exponentially with grid resolution
- Discretized maps fail to represent complex terrain features (ditches, irregular rocks)
- Path quality varies based on grid resolution/shape

RRT* overcomes these by working directly on LiDAR point cloud data (continuous space) and asymptotically converging to optimal paths.

### Algorithm Overview

The Traversability-Based RRT* extends conventional RRT* by incorporating rover traversability assessment during tree expansion.

#### Core Data Structures

**Rover Configuration (Node):**
```
q_i = (x_i, y_i, z_i, φ_i, θ_i, ψ_i)

where:
  (x, y, z)  = Center of gravity coordinates
  φ (phi)   = Roll angle
  θ (theta) = Pitch angle  
  ψ (psi)   = Yaw angle
```

#### Main Algorithm

```
Algorithm: Traversability-Based RRT*
Input:  Point cloud data (Pc) from LiDAR
Output: Tree (T) containing traversable path

1.  T.add_vertex(q_init)           // Add start position
2.  cost(q_init) = 0
3.  WHILE q_i ≠ q_goal DO:
4.      q_i ← GetRandomNode()      // Sample from point cloud
5.      FOR ALL q_j in T_near DO:
6.          IF TraversabilityAssessment(q_j, q_i) THEN:
7.              cost = cost(q_j) + CostFunction(q_j, q_i)
8.              IF cost < cost(q_i) THEN:
9.                  cost(q_i) = cost
10.                 parent(q_i) = q_j
11.                 q_n = q_j
12.         T.add_vertex(q_i)
13.         T.add_edge(q_n, q_i)
14.         
15.         // Rewiring phase (RRT* optimization)
16.         FOR ALL q_near in T_near except q_i DO:
17.             IF TraversabilityAssessment(q_i, q_near) THEN:
18.                 cost = cost(q_i) + CostFunction(q_i, q_near)
19.                 IF cost(q_near) > cost THEN:
20.                     T.rewire(q_i, q_near)
21.                     cost(q_near) = cost
22.                     parent(q_near) = q_i
23. RETURN T
```

#### Cost Function

The cost function evaluates path quality based on distance and rover orientation:

```
C(q_i) = W_L × (L_i / N_L) + W_φ × (φ_i / N_φ) + W_θ × (θ_i / N_θ) + W_ψ × (Δψ_i / N_ψ)

where:
  L_i    = Distance between nodes
  φ_i    = Roll angle at node i
  θ_i    = Pitch angle at node i  
  Δψ_i   = Yaw rotation from parent to node i
  W_*    = Weighting factors (default: 0.25 each)
  N_*    = Normalization factors
```

#### Traversability Assessment

The traversability check ensures safe path expansion by evaluating:

```
Algorithm: Traversability Assessment
Input:  q_j (parent), q_i (candidate)
Output: True/False

1. Check if q_i is within:
   - Heading-biased region (H_j): ±ψ_th from rover facing direction
   - Goal-biased region (G_j): ±90° from direction to goal
   - Radial bounds: R_min ≤ distance ≤ R_max
   
2. Check elevation angle: |δ_i| < δ_th

3. Calculate rover pose at q_i using wheel contact points:
   - Compute roll (φ) and pitch (θ) from terrain geometry
   
4. IF φ_i < φ_th AND θ_i < θ_th THEN:
      RETURN True
   ELSE:
      RETURN False
```

**Traversability Regions Visualization:**
```
                    Goal
                     ↑
                    /|\
                   / | \
        +---------/  |  \---------+
        |        /   |   \        |
        |   G_j /    |    \ G_j   |   ← Goal-biased region (±90°)
        |      /     |     \      |
        +-----/------●------\-----+
              \   q_j|H_j   /         ← Heading-biased region (±ψ_th)
               \     |     /
                \    ↑    /
                 \   |   /
                  \  |  /           R_max
                   \ | /            ↕
                    \|/             R_min
              [Sampling Area]
```

### Default Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| R_max | 0.830 m | Maximum sampling radius |
| R_min | 0.415 m | Minimum sampling radius |
| φ_th | 10.0° | Roll angle threshold |
| θ_th | 10.0° | Pitch angle threshold |
| ψ_th | 50.0° | Heading angle threshold |
| δ_th | 10.2° | Elevation angle threshold |

### Key Properties

1. **Asymptotic Optimality**: Path cost converges to optimal as sampling trials increase
2. **Continuous Space**: Works directly on point cloud data without discretization
3. **Safety-Aware**: Avoids steep slopes, obstacles, and occluded areas
4. **Goal-Biased**: Tree expansion is directed toward the goal
5. **Rewiring**: Continuously improves existing paths during exploration

### Performance

Based on simulation results from the reference paper:
- Converges to near-optimal paths within ~30,000 sampling trials
- Error rate vs optimal path: 0.4% - 9% depending on terrain size
- Outperforms conventional RRT with traversability assessment

## High Level Navigation Algorithm

The autonomous navigation follows this state machine:

```
┌─────────────────────────────────────────────────────────────┐
│  START: Rover placed at starting location                  │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│  1. Mark starting location as origin                       │
│  2. Set first waypoint/ArUco marker as target              │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
┌─────────────────────────────────────────────────────────────┐
│  3. Auto-navigate (explore) for set duration               │◄──┐
└─────────────────────────┬───────────────────────────────────┘   │
                          ▼                                       │
┌─────────────────────────────────────────────────────────────┐   │
│  4. Raise arm and rotate camera to search for waypoint     │   │
└─────────────────────────┬───────────────────────────────────┘   │
                          ▼                                       │
                    ┌───────────┐                                 │
                    │  Waypoint │──── No ─────────────────────────┘
                    │  Found?   │
                    └─────┬─────┘
                          │ Yes
                          ▼
┌─────────────────────────────────────────────────────────────┐
│  6. Generate RRT* path to waypoint                         │
│  7. Traverse path and dock at waypoint                     │
│  8. Mark waypoint as found                                 │
└─────────────────────────┬───────────────────────────────────┘
                          ▼
                    ┌───────────┐
                    │    All    │──── No ──► Set next waypoint ──┐
                    │  Found?   │                                │
                    └─────┬─────┘                                │
                          │ Yes                                  │
                          ▼                                      │
┌─────────────────────────────────────────────────────────────┐  │
│  10. Generate RRT* path back to origin                     │  │
│  11. Traverse path and STOP                                │  │
└─────────────────────────────────────────────────────────────┘  │
                                                                 │
                    ┌────────────────────────────────────────────┘
                    ▼
            (Return to step 4)
```

## Installation

### Prerequisites

1. ROS 2 Humble installed on Ubuntu 22.04
2. Required ROS packages:
   ```bash
   sudo apt-get install ros-humble-pcl-ros ros-humble-pcl-conversions ros-humble-visualization-msgs
   ```
3. Eigen3:
   ```bash
   sudo apt-get install libeigen3-dev
   ```
4. Unitree LiDAR SDK2 (for L2 LiDAR):
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