# dv\_slam

Monocular visual SLAM frontend for underwater and general-purpose datasets.
Extracts ORB features, estimates camera motion via essential-matrix initialisation
and PnP tracking, maintains a local map of 3-D landmarks with a covisibility
graph, and publishes trajectory + odometry (with covariance) over ROS 2.
<!-- 
## Package layout

```
dv_slam/
├── include/
│   ├── dv_slam/
│   │   └── utility.hpp            # Eigen ↔ OpenCV conversion helpers
│   └── frontend/
│       ├── VisualFrontend.hpp      # 3-stage state machine (INIT → TRACKING)
│       ├── PoseEstimator.hpp       # Essential, PnP + RANSAC, triangulation
│       ├── FeatureExtractor.hpp    # ORB extraction wrapper
│       ├── FeatureMatcher.hpp      # Descriptor & projection-based matching
│       ├── Frame.hpp               # Single image observation
│       ├── KeyFrame.hpp            # Sparse keyframe + covisibility graph
│       ├── MapPoint.hpp            # 3-D landmark
│       ├── Map.hpp                 # Global map container
│       ├── Viewer.hpp              # Live keypoint / match viewer
│       ├── Pose3d.hpp              # using Pose3d = Eigen::Isometry3d
│       └── VisualTypes.hpp         # ORBParams struct
├── src/
│   ├── frontend/                   # Library sources (dv_slam_frontend)
│   └── visual_odom/                # ROS 2 node (vo_node)
│       ├── visual_odom_node.hpp
│       ├── visual_odom_node.cpp
│       └── visual_odom_main.cpp
├── config/
│   ├── dv_slam.yaml                # Shared params (ORB, QoS, mono scale, …)
│   └── datasets/
│       ├── aqualoc.yaml            # Fisheye underwater (default)
│       ├── aquaslam.yaml           # Aquatic SLAM dataset
│       ├── oceansim.yaml           # Ocean simulation
│       ├── tum_fr2_rgb.yaml        # TUM Freiburg2 (pinhole)
│       └── tum_room4.yaml          # TUM Room4 (fisheye)
├── launch/
│   └── visual_odom.launch.py       # Launch with base + dataset overlay
├── test/
│   └── test_matching.cpp           # GTest: feature matching visualisation
├── CMakeLists.txt
└── package.xml
``` -->

## Prerequisites

- ROS 2 **Jazzy** (or compatible) workspace with `colcon`.
- Dependencies: `rclcpp`, `image_transport`, `cv_bridge`, `nav_msgs`,
  `geometry_msgs`, `sensor_msgs`, `OpenCV` (core, calib3d, imgproc, highgui),
  `Eigen3`.
- (Optional) [`evo`](https://github.com/MichaelGrupp/evo) for offline
  trajectory evaluation against ground truth.

## Building

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dv_slam --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

This produces:
- `dv_slam_frontend` — static library with all frontend classes.
- `vo_node` — ROS 2 executable that runs the visual odometry pipeline.

## Running

```bash
source install/setup.bash
ros2 launch dv_slam visual_odom.launch.py
```

The launch file loads `config/dv_slam.yaml` first, then overlays a
dataset-specific config. The default dataset is **aqualoc**.

To switch datasets:

```bash
ros2 launch dv_slam visual_odom.launch.py dataset_config:=datasets/tum_fr2_rgb.yaml
```

## Published topics

| Topic         | Type                      | Description                              |
|---------------|---------------------------|------------------------------------------|
| `trajectory`  | `nav_msgs/msg/Path`       | Accumulated camera path (all poses).     |
| `odometry`    | `nav_msgs/msg/Odometry`   | Per-frame pose with 6×6 covariance.      |

## Configuration

### Shared parameters (`config/dv_slam.yaml`)

| Parameter                   | Default          | Description                                          |
|-----------------------------|------------------|------------------------------------------------------|
| `matcher_type`              | `NORM_HAMMING`   | Descriptor norm (`NORM_HAMMING` or `NORM_L2`).       |
| `frontend.enable_viewer`    | `false`          | Show live keypoint / match window.                   |
| `frontend.cv_rng_seed`      | `0`              | OpenCV RNG seed (`0` = non-deterministic).           |
| `frontend.cv_num_threads`   | `1`              | OpenCV thread count (`0` = default).                 |
| `input.queue_size`          | `200`            | Image subscriber queue depth.                        |
| `input.reliable_qos`        | `true`           | Use reliable QoS (vs. best-effort).                  |
| `mono.init_scale`           | `1.0`            | Initial metric scale for monocular translation.      |
| `orb.*`                     | —                | ORB extractor settings (n\_features, scale\_factor, n\_levels, edge\_threshold, first\_level, wta\_k, score\_type, patch\_size, fast\_threshold). |
| `output.save_tum_trajectory`| `true`           | Write poses to a TUM-format text file.               |
| `output.tum_trajectory_path`| `vo_trajectory_tum.txt` | Output file path for TUM trajectory.          |

### Dataset overlay parameters (`config/datasets/*.yaml`)

| Parameter          | Description                                                |
|--------------------|------------------------------------------------------------|
| `image_topic`      | ROS image topic to subscribe to.                           |
| `Camera.model`     | `pinhole` (k1,k2,p1,p2,k3) or `fisheye` (k1..k4).       |
| `Camera.fx/fy/cx/cy` | Camera intrinsics.                                      |
| `Camera.k1..k4, p1, p2` | Distortion coefficients.                              |
| `Camera.width/height` | Image resolution (required for fisheye undistortion).   |
| `Camera.undistort` | Enable runtime undistortion (`true`/`false`).              |

## Trajectory evaluation

After running the node on a dataset:

```bash
# Absolute Pose Error
evo_ape tum vo_trajectory_tum.txt /path/to/groundtruth.txt \
    --align --correct_scale --plot

# Relative Pose Error
evo_rpe tum vo_trajectory_tum.txt /path/to/groundtruth.txt \
    -va --align --correct_scale -p --plot_mode=xyz

# Visual trajectory comparison
evo_traj tum vo_trajectory_tum.txt --ref=/path/to/groundtruth.txt \
    -p --plot_mode=xyz
```

## Architecture

The frontend implements a four-stage state machine:

1. **NO\_IMAGES\_YET** — waiting for the first image.
2. **INITIALIZING** — collecting two-view correspondences and estimating the
   essential matrix (2-D↔2-D). On success, triangulates an initial set of
   MapPoints and transitions to tracking.
3. **TRACKING** — for each new frame, propagates MapPoint observations from
   the reference KeyFrame, solves PnP (3-D↔2-D) with RANSAC, validates the
   pose jump, and optionally promotes the frame to a new KeyFrame
   (triggering triangulation and covisibility updates).
4. **LOST** — entered after `kMaxConsecutiveFailures` (5) failed tracking
   attempts. Attempts relocalization against known KeyFrames for up to
   `kMaxRelocFrames` (30) frames; if recovery fails, resets to
   NO\_IMAGES\_YET and starts a new map segment.

Each KeyFrame event produces a `FrontendOutput` with relative pose +
covariance, ready for consumption by an external GTSAM-based backend
optimizer (not included in this package).

## Notes

- The frontend assumes monocular input with unknown scale;
  `mono.init_scale` roughly scales translations before backend optimisation.
- Set `Camera.model` to `fisheye`/`equidistant` for equidistant distortion
  (k1–k4), or `pinhole` for standard radial-tangential (k1, k2, p1, p2, k3).
- Fisheye undistortion maps are precomputed once (lazily on first frame if
  `Camera.width/height` are not set).
- CLAHE contrast enhancement is applied to every frame before feature
  extraction.
- Set `frontend.enable_viewer` to `true` for real-time keypoint display
  (requires a GUI / OpenCV highgui).
