# Multi-Camera Rig GUI

ROS 2 PyQt5-based GUI for controlling and monitoring the multi-camera rig system. Provides a full-screen interface optimized for small displays with large controls, real-time image display, and parameter adjustment.

## Features

- **Full-Screen Interface**: Automatically expands to fill the screen
- **Large Start/Stop Button**: Toggle recording with visual feedback (grey → red)
- **5 Parameter Sliders**: Control flash duration, frame rate, camera exposures, and gain
- **Dual Image Display**: Real-time preview from Firefly and Ximea cameras (stacked vertically)
- **Status Log**: Monitor system messages and parameter changes
- **Calibration Buttons**: Dark and FFC calibration for Ximea camera
- **Responsive Controls**: Sliders only log final values on release

## Installation

### Dependencies

```bash
pip3 install PyQt5 opencv-python
sudo apt-get install ros-${ROS_DISTRO}-cv-bridge
```

### Build

```bash
cd ~/cmu/kantor_lab/ros2_ws
colcon build --packages-select multi_camera_rig_gui
source install/setup.bash
```

## Configuration

Edit `config/gui_params.yaml` to configure:

- **Image topics**: Camera image streams to display
- **Slider ranges**: Min/max/default values for each parameter
- **Node mappings**: Which ROS node controls each parameter
- **Services**: Trigger services for parameter application

## Usage

### Launch GUI

```bash
ros2 launch multi_camera_rig_gui gui.launch.py
```

### Complete System Startup

```bash
# Terminal 1: Trigger node
ros2 launch multi_camera_rig_trigger trigger.launch.py

# Terminal 2: Cameras
ros2 launch firefly-ros2-wrapper-bringup bringup_real.launch.py &
ros2 launch ximea-ros2-wrapper-bringup bringup_real.launch.py &

# Terminal 3: GUI
ros2 launch multi_camera_rig_gui gui.launch.py
```

## Interface Layout

### Left Panel (40% width)
- **START/STOP Button** - Toggle video recording
- **Flash Duration** - Trigger flash duration (0-300 ms)
- **Frame Rate** - Trigger frequency (1-20 Hz)
- **Firefly Exposure** - Firefly camera exposure time (μs)
- **Ximea Gain** - Ximea camera gain (dB)
- **Ximea Exposure** - Ximea camera exposure time (μs)
- **Calibration Buttons** - Dark Calibrate, FFC Calibrate

### Right Panel (60% width)
- **Firefly Camera** - Live image display
- **Ximea Camera** - Live image display
- **Status Messages** - Parameter updates and system messages

## Controls

### Recording
- Click **START** to begin recording (button turns red)
- Click **STOP** to end recording (button returns to grey)
- Services called: `/trigger/start_video`, `/trigger/stop_video`

### Sliders
- Drag slider to adjust value (label updates in real-time)
- Release slider to apply and log the change
- Flash Duration and Frame Rate automatically call trigger services

### Calibration
- **Dark Calibrate** - Placeholder for dark calibration implementation
- **FFC Calibrate** - Placeholder for FFC calibration implementation

## ROS 2 Interface

### Subscribed Topics
- `/firefly_left/image_raw` - Firefly camera image
- `/ximea/image_raw` - Ximea camera image

### Service Clients
- `/trigger/start_video` - Start video recording
- `/trigger/stop_video` - Stop video recording
- `/trigger/set_flash_duration` - Apply flash duration changes
- `/trigger/set_frame_rate` - Apply frame rate changes

### Parameters Set
- `/trigger_node/flash_duration_ms`
- `/trigger_node/frame_rate_hz`
- `/firefly_camera_node/shutter_time`
- `/ximea_camera_node/gain_db`
- `/ximea_camera_node/exposure_time_us`