# Multi-Camera Rig Trigger Controller

ROS 2 Python package for hardware trigger control in the multi-camera rig system. Provides serial communication interface with trigger hardware and integrates with the GUI.

## Features

- **Serial Communication**: USB serial interface at 9600 baud with proper configuration
- **Connection Testing**: Verify hardware connection with 'i' command (expects 'ih\r\n' response)
- **Single Trigger**: Send individual trigger pulses
- **Continuous Recording**: Start/stop continuous trigger sequences
- **Flash Duration Control**: Set flash duration (0-300 ms)
- **Frame Rate Control**: Set trigger frequency (1-20 Hz)
- **Joystick Control**: Trigger video with gamepad/joystick buttons
- **ROS 2 Services**: Control via service calls
- **GUI Integration**: Responds to director topic commands
- **Parameter Management**: Dynamic parameter updates

## Hardware Interface

The package communicates with trigger hardware via `/dev/ttyUSB0` using:
- **Baud rate**: 9600
- **Data bits**: 8
- **Parity**: None
- **Stop bits**: 1
- **Flow control**: None

### Hardware Commands

| Command | Description | Example |
|---------|-------------|---------|
| `i` | Test connection (expects 'ih\r\n') | `i` |
| `t` | Send single trigger | `t` |
| `r` | Start video recording | `r` |
| `s` | Stop video recording | `s` |
| `f<duration>` | Set flash duration (ms) | `f100` |
| `c<rate>` | Set frame rate (Hz) | `c10` |

## Installation

### Dependencies

Install pyserial:
```bash
pip3 install pyserial
```

### Build

```bash
cd ~/cmu/kantor_lab/ros2_ws
colcon build --packages-select multi_camera_rig_trigger
source install/setup.bash
```

## Usage

### Launch the Trigger Node

**Default configuration:**
```bash
ros2 launch multi_camera_rig_trigger trigger.launch.py
```

**Custom configuration:**
```bash
ros2 launch multi_camera_rig_trigger trigger.launch.py \
    serial_port:=/dev/ttyUSB1 \
    flash_duration_ms:=150 \
    frame_rate_hz:=15.0
```

### ROS 2 Services

#### Test Connection
```bash
ros2 service call /trigger/test_connection std_srvs/srv/Trigger
```

#### Send Single Trigger
```bash
ros2 service call /trigger/send_trigger std_srvs/srv/Trigger
```

#### Start Video Recording
```bash
ros2 service call /trigger/start_video std_srvs/srv/Trigger
```

#### Stop Video Recording
```bash
ros2 service call /trigger/stop_video std_srvs/srv/Trigger
```

#### Set Flash Duration (via parameter, then apply)
```bash
ros2 param set /trigger_node flash_duration_ms 150
ros2 service call /trigger/set_flash_duration std_srvs/srv/Trigger
```

#### Set Frame Rate (via parameter, then apply)
```bash
ros2 param set /trigger_node frame_rate_hz 15.0
ros2 service call /trigger/set_frame_rate std_srvs/srv/Trigger
```

### ROS 2 Topics

#### Status Messages
```bash
# Monitor trigger status
ros2 topic echo /trigger/status
```

#### Director Integration
The node subscribes to `/multi_cam_rig/director` and responds to:
- `"Capture N"` - Sends single trigger (if not recording)
- `"Recording started"` - Starts video recording
- `"Recording stopped"` - Stops video recording
- `"Flash duration: X"` - Updates flash duration to X ms

### Parameters

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| `serial_port` | string | `/dev/ttyUSB0` | - | Serial port device |
| `baudrate` | int | 9600 | - | Serial baud rate |
| `flash_duration_ms` | int | 100 | 0-300 | Flash duration (ms) |
| `frame_rate_hz` | double | 10.0 | 1-20 | Trigger rate (Hz) |
| `auto_connect` | bool | true | - | Test connection on startup |

**View current parameters:**
```bash
ros2 param list /trigger_node
ros2 param get /trigger_node flash_duration_ms
```

**Update parameters:**
```bash
ros2 param set /trigger_node flash_duration_ms 200
ros2 param set /trigger_node frame_rate_hz 5.0
```

## GUI Integration

The trigger node integrates seamlessly with `multi_camera_rig_gui`:

1. The GUI publishes commands to `/multi_cam_rig/director`
2. The trigger node subscribes and responds automatically
3. Status updates are published to `/trigger/status`

### Example Workflow

```bash
# Terminal 1: Start trigger node
ros2 launch multi_camera_rig_trigger trigger.launch.py

# Terminal 2: Start cameras
ros2 launch firefly-ros2-wrapper-bringup bringup_real.launch.py &
ros2 launch ximea-ros2-wrapper-bringup bringup_real.launch.py &

# Terminal 3: Start GUI
ros2 launch multi_camera_rig_gui gui.launch.py

# Now use GUI buttons to control triggers!
```

## Python API Usage

You can also use the hardware interface directly in Python:

```python
from multi_camera_rig_trigger.hardware_interface import TriggerHardwareInterface

# Create interface
trigger = TriggerHardwareInterface(port="/dev/ttyUSB0", baudrate=9600)

# Test connection
success, msg = trigger.test_connection()
print(f"Connection: {msg}")

# Send single trigger
success, msg = trigger.send_trigger()

# Start video recording
success, msg = trigger.start_video()

# Set flash duration to 150 ms
success, msg = trigger.set_flash_duration(150)

# Set frame rate to 15 Hz
success, msg = trigger.set_frame_rate(15.0)

# Stop video recording
success, msg = trigger.stop_video()

# Close connection
trigger.close_connection()
```

## Troubleshooting

### Joystick/Gamepad Control

Control video triggering using a gamepad (Xbox controller):

**Run joy node and joy_trigger_node:**
```bash
# Terminal 1: Start joy node (publishes /joy topic)
ros2 run joy joy_node

# Terminal 2: Start joy trigger node
ros2 run multi_camera_rig_trigger joy_trigger_node
```

**Or use with main trigger launch:**
```bash
ros2 launch multi_camera_rig_trigger trigger.launch.py use_joy:=true
```

**Button mapping (Xbox controller):**
- **A button**: Toggle video recording (press to start, release to stop)

**Custom button mapping:**
```bash
ros2 run multi_camera_rig_trigger joy_trigger_node --ros-args \
  -p trigger_button:=1  # Use B button (index 1) instead
```

**Test joystick input:**
```bash
ros2 topic echo /joy
```

## Troubleshooting

### Permission Denied on Serial Port

Add your user to the dialout group:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

Or give temporary permissions:
```bash
sudo chmod 666 /dev/ttyUSB0
```

### Connection Test Fails

1. Check hardware is connected:
   ```bash
   ls -l /dev/ttyUSB*
   ```

2. Verify correct port:
   ```bash
   dmesg | grep tty
   ```

3. Test with screen:
   ```bash
   screen /dev/ttyUSB0 9600
   # Type 'i' and press Enter
   # Should see 'ih' response
   # Exit with Ctrl+A, then K
   ```

### No Response from Hardware

- Ensure hardware is powered on
- Check USB cable connection
- Verify baud rate matches hardware (9600)
- Try unplugging and reconnecting USB

### Services Not Available

Check if node is running:
```bash
ros2 node list | grep trigger
ros2 node info /trigger_node
```

### Video Recording Won't Start

- Ensure no single trigger is being sent
- Check `/trigger/status` topic for error messages
- Verify hardware connection with test service

## File Structure

```
multi_camera_rig_trigger/
├── config/
│   └── trigger_params.yaml       # Default parameters
├── launch/
│   └── trigger.launch.py         # Launch file
├── multi_camera_rig_trigger/
│   ├── __init__.py
│   ├── hardware_interface.py    # Serial communication
│   └── trigger_node.py           # ROS 2 node
├── package.xml
├── setup.py
├── resource/
│   └── multi_camera_rig_trigger
└── README.md
```

## Safety Features

- **State Tracking**: Prevents single triggers during video recording
- **Parameter Validation**: Enforces valid ranges for duration and rate
- **Automatic Cleanup**: Stops recording on node shutdown
- **Connection Management**: Proper opening/closing of serial port
- **Error Handling**: Comprehensive error messages and logging

## Advanced Usage

### Custom Service Client

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class TriggerClient(Node):
    def __init__(self):
        super().__init__('trigger_client')
        self.cli = self.create_client(Trigger, '/trigger/send_trigger')
        
    def send_trigger(self):
        req = Trigger.Request()
        future = self.cli.call_async(req)
        return future

# Usage
rclpy.init()
client = TriggerClient()
future = client.send_trigger()
rclpy.spin_until_future_complete(client, future)
result = future.result()
print(f"Success: {result.success}, Message: {result.message}")
```

### Monitor All Trigger Activity

```bash
# Watch status updates
ros2 topic echo /trigger/status

# Monitor parameter changes
ros2 param list /trigger_node --monitor

# View all services
ros2 service list | grep trigger
```

## Contributing

When adding new hardware commands:
1. Add method to `hardware_interface.py`
2. Add corresponding service in `trigger_node.py`
3. Update parameter handling if needed
4. Add documentation to this README
5. Test thoroughly with hardware

## License

MIT
