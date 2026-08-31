# AprilTag Detector

A ROS 2 package for detecting AprilTags from camera feed and estimating their pose in 3D space.

## Overview

This package provides ROS 2 nodes that:
- Subscribe to camera image and camera info topics
- Detect AprilTags (Tag36h11 family) in the image
- Estimate the 3D pose (position and orientation) of detected tags
- Publish detected tag poses as `SharedControlGoalArray` messages for shared control applications
- Transform detected poses to a target frame (e.g., robot base_link) using TF2

## Features

- Real-time AprilTag detection using the apriltag library
- 3D pose estimation for each detected tag
- Configurable tag sizes and detection parameters
- Supports USB camera integration via usb_cam
- Flexible topic remapping for different camera setups
- TF2-based pose transformation to target frames
- Composable node architecture for efficient processing

## Dependencies

- ROS 2 (Humble or later)
- OpenCV
- apriltag
- Eigen3
- cv_bridge
- sensor_msgs
- geometry_msgs
- tf2
- tf2_ros
- tf2_geometry_msgs
- rclcpp
- rclcpp_components
- usb_cam (for camera input)
- extender_msgs (custom message types)

## Installation

Build the package with colcon:

```bash
sudo apt-get install ros-humble-usb-cam
sudo apt-get install ros-humble-apriltag ros-humble-apriltag-ros

cd /home/megane/ros2_humble_ws
colcon build --packages-select apriltag_detector
source install/setup.bash
```

## Usage

### Launch the detector with USB camera

Run the complete detection pipeline with USB camera input:

```bash
ros2 launch apriltag_detector explorer_camera_detection.launch.py
```

This will:
1. Start the USB camera node (configured for `/dev/video2` with calibration from `explorer_camera_calib.yaml`)
2. Start the AprilTag detector node
3. Start the AprilTag bridge node for pose transformation

### Run detector only

If you already have a camera node running:

```bash
ros2 run apriltag_detector apriltag_detector
```

## Configuration

### AprilTag Detector Config

Configure the detector in `config/tags_params.yaml`:

```yaml
apriltag_detector:
  ros__parameters:
    max_hamming_distance: 1          # Maximum Hamming distance for detection
    tag_sizes:
      "0": 0.024                     # Tag ID: size in meters
      "1": 0.024
      "2": 0.024
      # ... up to tag ID 9
```

Parameters:
- `max_hamming_distance`: Maximum acceptable Hamming distance for tag detection (higher = more permissive)
- `tag_sizes.<tag_id>`: Physical size of each tag in meters (required for pose estimation)

### AprilTag Bridge Config

The bridge node transforms detected tag poses to a target frame. Configure in the launch file:

```python
ComposableNode(
    package='apriltag_detector',
    plugin='vision_tools::AprilTagBridge',
    name='apriltag_bridge',
    parameters=[{
        'target_frame': 'base_link'  # Target frame for pose transformation
    }],
)
```

Parameters:
- `target_frame`: The TF2 frame to transform poses into (e.g., 'base_link')

## Topics

### Subscriptions

- `/image_raw` (sensor_msgs/Image): Raw camera image
- `/camera_info` (sensor_msgs/CameraInfo): Camera calibration information

### Publications

- `/tag_detections` (extender_msgs/SharedControlGoalArray): Array of detected tags with their 3D poses (detector node)
- `/shared_control/dynamic_goals` (extender_msgs/SharedControlGoalArray): Array of detected tags transformed to target frame (bridge node)

## Visual Servoing Integration

Robin's current `visual_servoing` node consumes the detector output directly:

| Topic | Message | Producer / consumer |
| --- | --- | --- |
| `/image_raw` | `sensor_msgs/msg/Image` | Camera node -> AprilTag detector |
| `/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera node -> AprilTag detector |
| `/tag_detections` | `extender_msgs/msg/SharedControlGoalArray` | AprilTag detector -> visual servoing |
| `/visual_servoing/velocity_command` | `geometry_msgs/msg/TwistStamped` | visual servoing -> UI monitor |
| `/visual_servoing/error_TAGtoTAGd` | `geometry_msgs/msg/TwistStamped` | visual servoing -> UI monitor |

The UI Topic Monitor should subscribe only to small diagnostic messages such as
`/tag_detections`, `/visual_servoing/velocity_command`, and
`/visual_servoing/error_TAGtoTAGd`. Do not monitor `/image_raw` or compressed image
topics there; use the webcam/video stream widget for visual feedback.

If the browser webcam preview is fluid but the ROS image pipeline makes AprilTag
processing too slow, the bottleneck is likely in the ROS camera/image transport path.
In that case, reading the camera directly inside the AprilTag detector node is a
reasonable temporary architecture, as long as the detector still publishes
`/tag_detections` with the same `SharedControlGoalArray` contract.

## Message Structure

The detector publishes `SharedControlGoalArray` messages containing:

```
goals: SharedControlGoal[]
  id: int32                  # AprilTag ID
  goal_pose: geometry_msgs/Pose
    position: Point (x, y, z)
    orientation: Quaternion (x, y, z, w)
```

The bridge node transforms these poses to the target frame and republishes them.
