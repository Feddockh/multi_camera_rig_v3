# Firefly ROS2 Wrapper - Reconstruction Package

This package provides 3D reconstruction capabilities for the Firefly flash stereo camera system.

## Architecture

The package follows a **modular architecture** with separation between ROS2 integration and core processing logic:

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 Node Layer                          │
│  (Thin wrappers - parameter handling, pub/sub)             │
├─────────────────────────────────────────────────────────────┤
│  foundation_stereo_    stereo_rectify_    semantic_         │
│  matcher_node          scale_node         pointcloud_node   │
└────────┬───────────────────┬────────────────────┬───────────┘
         │                   │                    │
         ▼                   ▼                    ▼
┌─────────────────────────────────────────────────────────────┐
│                  Processor Libraries                        │
│  (Core logic - testable without ROS2)                      │
├─────────────────────────────────────────────────────────────┤
│  FoundationStereo    StereoRectify    SemanticPointCloud   │
│  Matcher             Scale             (TODO)               │
└────────┬───────────────────┬────────────────────────────────┘
         │                   │
         ▼                   │
┌─────────────────┐          │
│  TrtRunner      │          │
│  (TensorRT)     │          │
└─────────────────┘          │
         │                   │
         └───────────┬───────┘
                     ▼
         ┌───────────────────────┐
         │   Shared Utilities    │
         │   - qos_utils         │
         │   - (future common)   │
         └───────────────────────┘
```

**Key Benefits:**
- **Testability**: Core processors testable without ROS2
- **Reusability**: Processors usable in non-ROS contexts
- **Consistency**: Shared QoS utilities across all nodes
- **Maintainability**: Clear separation of concerns

## Nodes

### 1. stereo_rectify_scale_node
**Purpose:** Performs stereo rectification and image scaling in a single pass for efficiency.

**Features:**
- Combines undistortion, rectification, and scaling operations
- Supports configurable output resolution
- Publishes both rectified images and scaled camera info
- Configurable QoS profiles for subscriber/publisher

**Topics:**
- Subscribes to: `image_raw`, `camera_info`
- Publishes: `image_rect_scaled`, `camera_info_rect_scaled`

### 2. foundation_stereo_matcher_node
**Purpose:** GPU-accelerated stereo matching using TensorRT-optimized foundation models.

**Features:**
- TensorRT inference for fast stereo matching
- Outputs: point clouds, depth images, disparity maps
- Advanced filtering options (speckle, median, bilateral, flying pixel removal)
- Configurable point cloud density via stride parameter
- Multiple filter modes for disparity, depth, and point cloud

**Topics:**
- Subscribes to: `left/image_rect_scaled`, `right/image_rect_scaled`, `left/camera_info`
- Publishes: `points` (PointCloud2), `depth` (Image), `disparity` (Image)

**Requirements:**
- CUDA Toolkit
- TensorRT (NvInfer)
- Pre-trained stereo matching model in TensorRT engine format

### 3. semantic_pointcloud (existing)
**Purpose:** Semantic segmentation integration with point clouds.

## Building

The package automatically detects TensorRT/CUDA availability:
- If found: Builds all nodes including `foundation_stereo_matcher_node`
- If not found: Builds only `stereo_rectify_scale_node` and `semantic_pointcloud`

```bash
cd ~/ros2_ws
colcon build --packages-select firefly-ros2-wrapper-reconstruction
```

## Usage

These nodes are typically launched via the parent bringup package launch files:
- `bringup_real.launch.py` - For real hardware
- `bringup_fake.launch.py` - For simulation/testing

## Dependencies

- rclcpp
- sensor_msgs
- cv_bridge
- message_filters
- image_geometry
- OpenCV
- CUDA Toolkit (optional, for foundation stereo)
- TensorRT (optional, for foundation stereo)

## Migration Note

This package was created by moving reconstruction-related nodes from `firefly-ros2-wrapper-bringup`:
- `foundation_stereo_matcher_node.cpp` - Moved from bringup
- `stereo_rectify_scale_node.cpp` - Moved from bringup
- Launch files updated to reference this package
