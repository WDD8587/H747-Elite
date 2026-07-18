# SLAM System Design

## Overview

The H747 Elite uses Google Cartographer for real-time SLAM on the RK3566 application processor. The system fuses data from a 2D lidar, 6-axis IMU, and wheel odometry to produce a consistent occupancy grid map and real-time pose estimates.

## Architecture

```
                    +--------------------------------------+
                    |         RK3566 Application           |
                    |                                      |
+----------+        |  +-----------+    +---------------+  |
| LDS Lidar +------>|--| Frontend  |--->|  Backend      |  |
| (6 Hz)    |        |  | ScanMatch |    | PoseGraphOpt  |  |
+----------+        |  +-----------+    +-------+-------+  |
                    |        |                    |        |
+----------+        |  +-----------+    +-------+-------+  |
| ICM-20948+------>|--| IMU       |    | Occupancy     |  |
| IMU      |        |  | Tracker   |    | Grid Map      |  |
| (1 kHz)  |        |  +-----------+    +-------+-------+  |
+----------+        |                            |        |
                    |                            v        |
+----------+        |  +-----------+    +---------------+  |
| Wheel     +------>|--| Odom      |    | Map Server    |  |
| Odometry |        |  | Tracker   |    | (REST API)    |  |
| (100 Hz) |        |  +-----------+    +---------------+  |
+----------+        +--------------------------------------+
                               |
                               v
                    +----------------------+
                    | Navigation Stack     |
                    | (move_base / DWA)    |
                    +----------------------+
```

## Sensor Specifications and Rates

| Sensor | Type | Rate | Resolution | Latency | Interface |
|--------|------|------|------------|---------|-----------|
| Lidar | 2D LDS (triangulation) | 6 Hz | 360 pts/scan, 1 degree | 30 ms | UART 3Mbps |
| IMU | ICM-20948 (6-axis) | 1 kHz | 16-bit accel, 16-bit gyro | 2 ms | SPI 10MHz |
| Odometry | Wheel encoder | 100 Hz | 360 CPR -> 0.61 mm/count | 5 ms | Shared mem |

### Lidar (LDS) Details

```
Type:            Triangulation-based laser distance sensor
Range:           0.15m to 8.0m
Accuracy:        +/- 15 mm (< 1m), +/- 2% (> 1m)
Sample Rate:     2160 samples/sec (6 Hz * 360 pts)
Wavelength:      785 nm (infrared)
Motor Speed:     288 RPM (controlled by PID, tachometer feedback)
Signal:          UART at 3,000,000 baud, 8N1
Protocol:        Custom packet: 0xAA 0x55 + 4 bytes per point
Point Format:    [quality(1B), angle_low(1B), angle_high(1B), dist_low(1B), dist_high(1B)]
                 angle = (angle_high << 8 | angle_low) / 64.0 degrees
                 distance = (dist_high << 8 | dist_low) / 10.0 mm
```

### IMU (ICM-20948) Details

```
Accelerometer:
  Range:          +/- 16G (default +/- 4G for navigation)
  Noise:          230 ug/sqrt(Hz)
  Non-linearity:  0.5%

Gyroscope:
  Range:          +/- 2000 dps (default +/- 500 dps for navigation)
  Noise:          6.6 mdps/sqrt(Hz)
  Non-linearity:  0.1%

Communication:    SPI mode 0, 10 MHz
Data Ready:       INT pin (PC0), interrupt at 1 kHz
Filtering:        On-chip DLPF enabled, accel BW 218 Hz, gyro BW 200 Hz
```

### Wheel Odometry Details

```
Encoder Type:     Magnetic, AMS22S, 12 PPR
Quadrature:       x4 decoding -> 48 counts/rev motor
Gear Ratio:       30:1 -> 1440 counts/rev wheel
Wheel Diameter:   70 mm -> 0.61 mm / count
Max Count Rate:   120,000 counts/sec (at 5000 RPM motor)
Kinematics:       Differential drive
  Linear Vel:     v = (omega_r + omega_l) * r_wheel / 2
  Angular Vel:    omega = (omega_r - omega_l) * r_wheel / wheel_base
  Wheel Base:     320 mm (center-to-center)
```

## Cartographer Frontend

### Configuration

```lua
-- cartographer.lua configuration excerpt
options = {
  map_builder = {
    use_map_builder = true,
    num_background_threads = 4,
  },
  trajectory_builder = {
    use_imu_data = true,
    use_odometry_data = true,
    min_range = 0.15,
    max_range = 8.0,
    min_z = -0.1,
    max_z = 0.5,
    missing_data_ray_length = 5.0,
    num_accumulated_range_data = 1,
    voxel_filter_size = 0.025,
    submaps = {
      resolution = 0.05,
      num_range_data = 90,
      range_data_inserter = {
        hit_probability = 0.55,
        miss_probability = 0.49,
        insert_free_space = true,
      },
    },
    ceres_scan_matcher = {
      occupied_space_weight_0 = 30.0,
      translation_weight = 10.0,
      rotation_weight = 10.0,
      ceres_solver_options = {
        use_nonmonotonic_steps = true,
        max_num_iterations = 10,
        num_threads = 1,
      },
    },
    real_time_correlative_scan_matcher = {
      linear_search_window = 0.1,
      angular_search_window = math.rad(20.),
      translation_delta_cost_weight = 1e-1,
      rotation_delta_cost_weight = 1e-1,
    },
    motion_filter = {
      max_time_seconds = 5.0,
      max_distance_meters = 0.2,
      max_angle_radians = math.rad(1.0),
    },
    imu_gravity_time_constant = 10.0,
    pose_extrapolator = {
      use_imu_based = true,
      constant_velocity = {
        imu_gravity_time_constant = 10.0,
      },
    },
  },
}
```

### Scan Matching Pipeline

1. **Raw Lidar Scan** (6 Hz)
   - Voxel filter (2.5 cm) to reduce noise and point count
   - Remove points outside min/max range
   - Transform from lidar frame to base_link frame (extrinsic calibration)

2. **Pose Extrapolation** (IMU + odometry, 100 Hz)
   - IMU provides angular velocity integration
   - Wheel odometry provides linear velocity estimate
   - Combined via complementary filter for 100 Hz pose prediction
   - Used as initial guess for scan matcher

3. **Real-Time Correlative Scan Matcher (CSM)**
   - Search window: 0.1m translation, 20 degrees rotation
   - Score each candidate alignment against submap probability grid
   - Select highest-scoring candidate as initial estimate
   - Provides robustness against large pose errors (kidnapped robot recovery)

4. **Ceres Scan Matcher (Nonlinear Optimization)**
   - Refine pose estimate from CSM using Ceres solver
   - Cost function: negative log probability of scan points in submap
   - Optimizes: (x, y, theta)
   - Constraints: 10 iterations maximum
   - Produces: Optimal pose estimate inserted into submap

### Submap Management

```
Submap Resolution:   5 cm
Submap Size:         100 m^2 max (200x200 cells, ~10m x 10m)
Points per Submap:   90 lidar scans (15 seconds at 6 Hz)
Insertion Condition: Motion filter (0.2m or 1 deg or 5 sec)

Submap Storage:
  - Probability grid: float32 per cell (0 = free, 0.5 = unknown, 1 = occupied)
  - Grid compressed: p*(1-p) for efficient range-based updates
  - Update formula: M_new(x) = clamp(odds^-1(odds(M_old(x)) * odds(p_hit)))
  - p_hit = 0.55, p_miss = 0.49
```

## Cartographer Backend

### Pose Graph Optimization

```
Optimization Engine: Ceres Solver
Solver Type:         SPARSE_NORMAL_CHOLESKY
Line Search:         LEVENBERG_MARQUARDT
Max Iterations:      50
Convergence:         Function tolerance 1e-6

Optimization Frequency:
  - Always: after every 5 submaps completed
  - On demand: when loop closure candidate found
  - Smoothed: post-optimization, update all poses via interpolation

Nodes in Graph:
  - Submap nodes: submaps with anchored pose (~every 90 scans)
  - Scan nodes: individual scan poses (~every scan, 6 Hz)
  - Constraints: between scan and submap (rigid), between scans (odometry)

Constraint Types:
  1. Odometry constraints: scan[i] -> scan[i+1] from wheel encoders
     - Information matrix: diag([100, 100, 1000]) (high rotation confidence)
     - Covariance model: translational sigma = 0.01m, rotational sigma = 0.005 rad
   
  2. IMU constraints: scan[i] -> scan[i+1] from IMU preintegration
     - Information matrix: diag([50, 50, 500])
     - Used primarily for rotation (gyro drift bounded)
   
  3. Loop closure constraints: scan[i] -> submap[j], where |i - j| > threshold
     - Generated by loop closure search
     - Information matrix: diag([300, 300, 3000]) (high confidence)
     - Threshold: at least 50 submaps apart or 10m distance
```

### Loop Closure

```
Search Strategy: Branch-and-bound (BBS)
  - Grid resolution levels: 4 (coarse to fine)
  - Score threshold: 0.7 (minimum match score)
  - Window: 10m x 10m, 60 degrees

Loop Closure Conditions:
  1. Robot revisits previously mapped area
  2. Current scan matches an existing submap with score > 0.7
  3. Constraint connects nodes that are > 50 submaps apart temporally

After Loop Closure:
  - Trigger global optimization
  - Propagate pose corrections to all nodes
  - Update visualization (if active)
```

## Map Representation

### Occupancy Grid

```
Resolution:   5 cm per cell
Dimensions:   Dynamic, expandable (max 100m x 100m -> 2000x2000 cells)
Storage:      1 byte per cell (0-100 = occupancy probability, 255 = unknown)
Compression:  Run-length encoding for network transmission
Coordinate:   ROS convention: x forward, y left, z up
Origin:       Robot start position (0, 0, 0)
```

### Map Server (REST API)

The map is served to the navigation stack via a local REST API on the RK3566:

| Endpoint | Method | Response | Rate Limit |
|----------|--------|----------|------------|
| /map | GET | Occupancy grid (PNG + YAML metadata) | 1 Hz |
| /map/metadata | GET | Resolution, origin, dimensions | 10 Hz |
| /pose | GET | Current robot pose (x, y, theta) | 100 Hz |
| /submaps | GET | List of submap IDs and corners | 1 Hz |
| /optimize | POST | Trigger global optimization | On demand |

### Map Saving

```
Auto-Save Triggers:
  - Every 5 minutes during cleaning
  - On docking station arrival
  - On low battery (before shutdown)
  - On manual save command

Format:     PBStream (Cartographer native) + PNG + YAML
Location:   /data/maps/<timestamp>/ (on RK3566 eMMC)
Retention:  Last 20 maps (oldest auto-deleted)
```

## Coordinate Frames

```
map -> odom -> base_link -> lidar
                        -> imu_link
                        -> left_wheel
                        -> right_wheel

Frames:
  map:         Global reference frame, origin at robot start
  odom:        Odometry frame, drift accumulates
  base_link:   Robot center, x forward, y left, z up
  lidar:       Lidar center, offset from base_link
  imu_link:    IMU center, offset from base_link

Transforms (static):
  base_link -> lidar:    x=+0.08m, y=0.0m, z=+0.05m, roll=0, pitch=0, yaw=0
  base_link -> imu_link: x=+0.02m, y=0.0m, z=+0.02m, roll=0, pitch=0, yaw=0
  base_link -> wheel:    x=0.0m, y=+-0.16m, z=0.0m

Extrinsic Calibration:
  - Lidar-to-base-link: Manual measurement + hand-eye calibration script
  - IMU-to-base-link: 6-face static calibration (factory step)
  - Wheel-to-base-link: Manual measurement, verified by UMBmark test
```

## Performance Budget

| Operation | Latency Budget | Typical Load |
|-----------|---------------|--------------|
| Scan reception + filtering | 15 ms | 8 ms |
| Scan matching (CSM + Ceres) | 50 ms | 35 ms |
| Submap insertion | 10 ms | 5 ms |
| Pose graph optimization | 1000 ms (async) | 200 ms |
| Loop closure detection | 200 ms (async) | 50 ms |
| Map publishing | 10 ms | 2 ms |
| **Total per scan cycle (166 ms)** | **< 150 ms** | **100 ms** |

CPU: RK3566 Quad-Core A55 @ 1.8 GHz  
Memory Usage: ~120 MB (map + pose graph + submaps)  
Threading: 1 main thread + 4 background threads (Cartographer) + 1 map publisher

## Navigation Integration

The output of Cartographer SLAM feeds into the ROS 2 navigation stack:

```
Cartographer                Navigation Stack
  |                              |
  |-- /map (occupancy grid) ---->|-- global_costmap
  |-- /tf (map->odom) ---------->|-- tf_listener
  |-- /pose -------------------->|-- amcl (filtered pose)
  |                              |
  |                              |-- move_base
  |                              |   |-- global_planner (NavFn)
  |                              |   |-- local_planner (DWA)
  |                              |
  |                              |-- /cmd_vel (to motor controller)

Planning Frequency: 10 Hz (global), 20 Hz (local)
Path Tolerance: 0.1m (global), 0.05m (local)
Goal Tolerance: 0.15m position, 0.2 rad rotation
```

## Failure Modes and Mitigation

| Failure Mode | Detection | Mitigation |
|-------------|-----------|------------|
| Lidar blocked | Scan points truncated or missing | Retry 3 scans; if persistent, rotate robot to clear |
| Lidar motor stall | Rotation speed out of tolerance (288 +/- 10 RPM) | Restart motor; if persistent, try cleaning mode with ToF only |
| IMU data loss | 100 consecutive missed IMU readings | Fall back to odometry-only pose estimation |
| Wheel slip | Odometry velocity inconsistent with IMU acceleration | Downweight odometry information matrix temporarily |
| Kidnapped robot | Large pose jump > 1m in one scan | Reset global localization, re-initialize from scan match |
| Pure rotation failure | No translation for > 5 seconds | Adapt search window, enable aggressive rotation matching |
| Memory pressure | Submap count > 200 | Begin submap pruning (drop oldest submaps) |
