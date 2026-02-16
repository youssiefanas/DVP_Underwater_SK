# DVP_Underwater_SK



## Visual Odometry frontend

- `src/frontend/*`: Visual frontend (feature extractor/matcher, frame/keyframe/map abstractions, pose estimator with essential/PnP, and the trajectory-producing `VisualFrontend` state machine).
- `src/visual_odom/*`: ROS 2 node that configures the frontend, subscribes to an image topic, publishes the path, and logs TUM poses (`vo_trajectory_tum.txt` in the repo root by default).
- `config/dv_slam.yaml`: Single point of truth for camera intrinsics, ORB parameters, frontend options (deterministic OpenCV seed/threads), and trajectory logging toggles.
- `launch/visual_odom.launch.py`: Launches `vo_node` with `dv_slam.yaml` and allows overriding the input topic.

<!-- ## Prerequisites

- ROS 2 `kilted` (or compatible) workspace with `colcon` and dependencies installed (`OpenCV`, `GTSAM`, `rclcpp`, `image_transport`, etc.).
- `evo` (or similar) for offline trajectory evaluation against ground truth.
- A dataset such as TUM `freiburg2_xyz` with RGB images (`config/rgb-d/associations/fr2_xyz.txt` contains the timestamps used during debugging). -->

## Building

```bash
source /opt/ros/kilted/setup.bash
colcon build --merge-install --packages-select dv_slam --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

The build produces the `dv_slam_frontend` library and the `vo_node` executable.

## Running

```bash
source install/setup.bash
ros2 launch dv_slam visual_odom.launch.py image_topic:=/camera/rgb/image_color
```

- The node reads `config/dv_slam.yaml` for camera intrinsics and frontend tuning.
- The `output.save_tum_trajectory` parameter controls logging to `vo_trajectory_tum.txt`; set `output.tum_trajectory_path` if you prefer another location.


## Trajectory evaluation

1. Run the node until the dataset finishes; the TUM trajectory file is overwritten each run.
2. Use `evo` to align/compare:
   ```bash
   evo_ape tum src/frontend/vo_trajectory_tum.txt path/to/GroundTruth.txt --align --correct_scale --plot
   evo_rpe tum src/frontend/vo_trajectory_tum.txt path/to/GroundTruth.txt --align --correct_scale
   evo_traj tum src/frontend/vo_trajectory_tum.txt --ref=path/to/GroundTruth.txt -p --plot_mode=xyz
   ```

<!-- 
## Testing

- `test/test_matching.cpp` contains a basic matcher visualization test; build and run with:
  ```bash
  colcon test --packages-select dv_slam
  colcon test-result --verbose
  ``` -->

## Notes

- The frontend assumes monocular input with unknown scale; `mono.init_scale` in `config/dv_slam.yaml` is useful for roughly scaling translations to the dataset before backend optimization.
- Set `frontend.enable_viewer` to `true` if you want the real-time keypoint display (requires GUI/OpenCV windows).
<!-- - `VisualFrontend` publishes `ja` path items whenever tracking succeeds; if you see large jumps, inspect `diag` output for pose jumps and `FeatureMatcher`. -->
