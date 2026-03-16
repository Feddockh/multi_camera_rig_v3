# Ximea ROS2 Wrapper Bringup

This package provides a ROS2 wrapper for Ximea cameras using the xiApi (m3api) interface.

## Features

- Direct interface with Ximea cameras via xiApi
- **Hardware trigger mode** for synchronized multi-camera capture
- Software flat-field correction (FFC) for image calibration
- Camera info manager integration for calibration
- Configurable exposure, gain, and other camera parameters
- Continuous capture thread waiting for hardware trigger signals

## Dependencies

- Ximea Software Package (xiAPI)
  - Install from: https://www.ximea.com/support/wiki/apis/XIMEA_Linux_Software_Package
  - Typically installed to `/opt/XIMEA/`
- ROS2 packages: rclcpp, sensor_msgs, cv_bridge, camera_info_manager

## Installation

1. Install Ximea SDK following manufacturer instructions
2. Build the package:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select ximea-ros2-wrapper-bringup
   source install/setup.bash
   ```

## Usage

### Basic Launch

```bash
ros2 launch ximea-ros2-wrapper-bringup bringup_real.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch ximea-ros2-wrapper-bringup bringup_real.launch.py \
  camera_name:=ximea_cam1 \
  device_id:=0 \
  gain:=2.0 \
  exposure_time:=15000 \
  trigger_timeout_ms:=10000
```

## Parameters

- `camera_name` (string, default: "ximea_camera"): Name of the camera
- `device_id` (int, default: 0): Ximea device ID
- `frame_id` (string, default: "ximea_optical_frame"): TF frame ID
- `trigger_timeout_ms` (int, default: 5000): Timeout for waiting for hardware trigger (ms)
- `gain` (double, default: 0.0): Camera gain in dB (-1.5 to 6.0)
- `exposure_time` (int, default: 10000): Exposure time in microseconds
- `enable_ffc` (bool, default: true): Enable flat field correction
- `ffc_dir` (string, default: ""): Directory containing FFC calibration files
- `camera_info_url` (string): URL to camera calibration YAML file

## Flat Field Correction (FFC)

The node supports software-based flat field correction using dark and mid (flat) field images.

Calibration files should be stored in `{ffc_dir}/` with naming:
- `YYYYMMDD_HHMMSS_dark.tif`: Dark field image
- `YYYYMMDD_HHMMSS_mid.tif`: Mid/flat field image

The node will automatically load the most recent calibration files on startup.

To reload calibration files at runtime without restarting the node, call the `reload_ffc` service:

```bash
ros2 service call /{camera_name}/reload_ffc std_srvs/srv/Trigger
```

## Topics

### Published

- `/{camera_name}/image_raw` (sensor_msgs/Image): Raw camera images
- `/{camera_name}/camera_info` (sensor_msgs/CameraInfo): Camera calibration info

## Services

- `/{camera_name}/reload_ffc` (std_srvs/Trigger): Reload FFC calibration files from `ffc_dir` at runtime

## Hardware Trigger Mode

The camera is configured for **hardware trigger mode** with the following settings:

- **GPI 1**: Configured as trigger input
- **Trigger Source**: Falling edge (XI_TRG_EDGE_FALLING)
- **GPO 1**: Set to exposure active output signal

The node runs a dedicated capture thread that continuously waits for hardware trigger signals. When a trigger is received:
1. Camera captures an image based on the trigger
2. Image is retrieved via xiGetImage with configurable timeout
3. Optional FFC is applied
4. Image and camera info are published to ROS topics

This mode is ideal for synchronized multi-camera systems where all cameras share a common hardware trigger signal.

## Notes

- Ensure proper permissions for USB devices: `sudo usermod -aG plugdev $USER`
- Camera calibration files should be generated using standard ROS calibration tools
- For multiple cameras, launch separate nodes with different device IDs
