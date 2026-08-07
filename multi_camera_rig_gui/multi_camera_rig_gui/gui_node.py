#!/usr/bin/env python3
"""
Multi-Camera Rig GUI Node

Provides a full-screen GUI for controlling the multi-camera rig system with:
- Large start/stop recording button
- 6 parameter sliders (flash duration, frame rate, camera gains/shutters)
- Real-time image display from two cameras
- Status message output area
"""

import sys
import signal
import subprocess
import os
import time
from datetime import datetime
import numpy as np
from typing import Dict, Any, Optional

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters, GetParameters
from rcl_interfaces.msg import Parameter as RclParameter, ParameterValue, ParameterType
from cv_bridge import CvBridge

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QSlider, QLabel, QTextEdit, QSplitter
)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QImage, QPixmap, QPalette, QColor

from multi_camera_rig_gui.gui_widgets import (
    create_action_button, create_utility_button, create_image_panel, create_slider_widget
)


class ROSSignals(QObject):
    """Signals for thread-safe GUI updates from ROS callbacks"""
    image_img1 = pyqtSignal(np.ndarray)
    image_img2 = pyqtSignal(np.ndarray)
    log_message = pyqtSignal(str)


class CameraRigGUI(QMainWindow):
    """Main GUI window for camera rig control"""
    
    def __init__(self, node: Node):
        super().__init__()
        self.node = node
        self.bridge = CvBridge()
        self.signals = ROSSignals()
        self.recording = False
        self.bag_recording = False
        self.bag_process: Optional[subprocess.Popen] = None
        self.current_bag_path: Optional[str] = None
        self._last_video_state_check = 0.0
        self._calibration_mode = None   # None | 'dark' | 'ffc'
        self._calibration_frames = []
        self._calibration_timeout_timer = None
        
        # Load parameters from config
        self.load_parameters()
        
        # Setup UI
        self.init_ui()
        
        # Connect signals
        self.signals.image_img1.connect(self.update_img1_image)
        self.signals.image_img2.connect(self.update_img2_image)
        self.signals.log_message.connect(self.append_log)
        
        # Setup ROS subscriptions and services
        self.setup_ros_interfaces()
        
        # Track which nodes have been configured
        self.configured_nodes = set()
        
        # Check if video is already running on startup
        QTimer.singleShot(500, self.check_recording_state)
        
        # Periodically check for new nodes and sync parameters
        self.node_sync_timer = QTimer()
        self.node_sync_timer.timeout.connect(self.check_and_sync_nodes)
        self.node_sync_timer.start(2000)  # Check every 2 seconds
        
        # Timer for ROS spinning
        self.ros_timer = QTimer()
        self.ros_timer.timeout.connect(self.spin_ros)
        update_rate = self.node.get_parameter('update_rate_hz').value
        self.ros_timer.start(int(1000 / update_rate))
        
        self.log("GUI initialized successfully")
    
    def load_parameters(self):
        """Load configuration parameters from ROS parameter server"""
        # Declare parameters with appropriate default values
        # The parameters will be loaded from the YAML file automatically
        
        # Declare basic parameters
        self.node.declare_parameter('window_title', 'Multi-Camera Rig Control')
        self.node.declare_parameter('update_rate_hz', 60.0)
        self.node.declare_parameter('trigger_start_service', '/trigger/start_video')
        self.node.declare_parameter('trigger_stop_service', '/trigger/stop_video')
        self.node.declare_parameter('trigger_is_running_service', '/trigger/is_video_running')
        self.node.declare_parameter('ffc_dir', os.path.expanduser('~/ffc_calibration'))
        self.node.declare_parameter('ximea_reload_ffc_service', '/ximea/reload_ffc')
        self.node.declare_parameter('force_default_params', False)
        self.node.declare_parameter('gui_config_file_path', '')
        self.node.declare_parameter('image_sub_qos_reliability', 'reliable')
        
        # Declare recording parameters
        self.node.declare_parameter('recording.topics', ['/firefly_left/image_raw', '/ximea/image_raw'])
        self.node.declare_parameter('recording.storage_path', '~/ros2_bags')
        self.node.declare_parameter('recording.storage_id', 'sqlite3')
        
        # Declare image topics
        self.node.declare_parameter('image_topics.img1', '/firefly_left/image_raw')
        self.node.declare_parameter('image_topics.img2', '/ximea/image_raw')
        
        # Declare flash duration parameters
        self.node.declare_parameter('flash_duration.min', 0)
        self.node.declare_parameter('flash_duration.max', 300)
        self.node.declare_parameter('flash_duration.default', 100)
        self.node.declare_parameter('flash_duration.node', '/trigger_node')
        self.node.declare_parameter('flash_duration.param', ['flash_duration_ms'])
        self.node.declare_parameter('flash_duration.service', '/trigger/set_flash_duration')
        
        # Declare frame rate parameters
        self.node.declare_parameter('frame_rate.min', 1)
        self.node.declare_parameter('frame_rate.max', 5)
        self.node.declare_parameter('frame_rate.default', 5)
        self.node.declare_parameter('frame_rate.node', '/trigger_node')
        self.node.declare_parameter('frame_rate.param', ['frame_rate_hz'])
        self.node.declare_parameter('frame_rate.service', '/trigger/set_frame_rate')
        
        # Declare firefly exposure parameters
        self.node.declare_parameter('firefly_exposure.min', 29)
        self.node.declare_parameter('firefly_exposure.max', 10000)
        self.node.declare_parameter('firefly_exposure.default', 10000)
        self.node.declare_parameter('firefly_exposure.node', '/firefly_camera_node')
        self.node.declare_parameter('firefly_exposure.param', ['shutter_time'])
        
        # Declare ximea gain parameters
        self.node.declare_parameter('ximea_gain.min', -1.5)
        self.node.declare_parameter('ximea_gain.max', 6.0)
        self.node.declare_parameter('ximea_gain.default', 0.0)
        self.node.declare_parameter('ximea_gain.step', 0.1)
        self.node.declare_parameter('ximea_gain.node', '/ximea_camera_node')
        self.node.declare_parameter('ximea_gain.param', ['gain_db'])
        
        # Declare ximea exposure parameters
        self.node.declare_parameter('ximea_exposure.min', 50)
        self.node.declare_parameter('ximea_exposure.max', 500000)
        self.node.declare_parameter('ximea_exposure.default', 10000)
        self.node.declare_parameter('ximea_exposure.node', '/ximea_camera_node')
        self.node.declare_parameter('ximea_exposure.param', ['exposure_time_us'])
        
        # Store slider configurations (all sliders now in left panel)
        self.slider_configs = {
            'Flash Duration (ms)': self.get_slider_config('flash_duration'),
            'Frame Rate (Hz)': self.get_slider_config('frame_rate'),
            'Firefly Exposure (μs)': self.get_slider_config('firefly_exposure'),
            'Ximea Gain (dB)': self.get_slider_config('ximea_gain'),
            'Ximea Exposure (μs)': self.get_slider_config('ximea_exposure'),
        }
    
    def get_slider_config(self, base_name: str) -> Dict[str, Any]:
        """Extract slider configuration from parameters"""
        config = {
            'base_name': base_name,
            'min': self.node.get_parameter(f'{base_name}.min').value,
            'max': self.node.get_parameter(f'{base_name}.max').value,
            'default': self.node.get_parameter(f'{base_name}.default').value,
            'node': self.node.get_parameter(f'{base_name}.node').value,
            'param': self.node.get_parameter(f'{base_name}.param').value,
        }
        # Convert single param to list for uniform handling
        if isinstance(config['param'], str):
            config['param'] = [config['param']]
        
        # Add optional step parameter (for non-integer sliders)
        if self.node.has_parameter(f'{base_name}.step'):
            config['step'] = self.node.get_parameter(f'{base_name}.step').value
        else:
            config['step'] = None
        
        # Add optional service parameter
        if self.node.has_parameter(f'{base_name}.service'):
            config['service'] = self.node.get_parameter(f'{base_name}.service').value
        else:
            config['service'] = None
        return config
    
    def init_ui(self):
        """Initialize the user interface"""
        # Set window properties
        title = self.node.get_parameter('window_title').value
        self.setWindowTitle(title)
        
        # Set background color to light grey
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(220, 220, 220))
        self.setPalette(palette)
        
        # Show maximized (full screen)
        self.showMaximized()
        
        # Main widget and layout
        main_widget = QWidget()
        self.setCentralWidget(main_widget)
        main_layout = QHBoxLayout(main_widget)
        
        # Create splitter for three columns
        splitter = QSplitter(Qt.Horizontal)
        main_layout.addWidget(splitter)
        
        # Left panel (30%)
        left_panel = self.create_left_panel()
        splitter.addWidget(left_panel)
        
        # Center panel (30%)
        center_panel = self.create_center_panel()
        splitter.addWidget(center_panel)
        
        # Right panel (40%)
        right_panel = self.create_right_panel()
        splitter.addWidget(right_panel)
        
        # Set sizes: left 30%, center 30%, right 40%
        splitter.setSizes([300, 300, 400])
    
    def create_left_panel(self) -> QWidget:
        """Create left panel with start/record buttons, status messages, and calibration buttons"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # Buttons container (30% of height)
        buttons_container = QWidget()
        buttons_layout = QVBoxLayout(buttons_container)
        buttons_layout.setContentsMargins(0, 0, 0, 0)
        buttons_layout.setSpacing(5)
        
        # Start/Stop button
        self.start_button = create_action_button("START")
        self.start_button.clicked.connect(self.toggle_recording)
        buttons_layout.addWidget(self.start_button)
        
        # Record button
        self.record_button = create_action_button("RECORD")
        self.record_button.clicked.connect(self.toggle_bag_recording)
        buttons_layout.addWidget(self.record_button)
        
        layout.addWidget(buttons_container, stretch=30)
        
        # Status messages (40% of height)
        status_container = QWidget()
        status_layout = QVBoxLayout(status_container)
        status_layout.setContentsMargins(0, 0, 0, 0)
        status_layout.setSpacing(5)
        
        log_label = QLabel("Status Messages:")
        log_label.setStyleSheet("font-size: 14px; font-weight: bold;")
        status_layout.addWidget(log_label)
        
        self.log_output = QTextEdit()
        self.log_output.setReadOnly(True)
        self.log_output.setStyleSheet("""
            QTextEdit {
                background-color: white;
                font-family: monospace;
                font-size: 11px;
                border: 2px solid #808080;
                border-radius: 5px;
            }
        """)
        status_layout.addWidget(self.log_output)
        
        layout.addWidget(status_container, stretch=40)
        
        # Calibration buttons (30% of height)
        button_container = QWidget()
        button_layout = QVBoxLayout(button_container)
        button_layout.setContentsMargins(0, 0, 0, 0)
        button_layout.setSpacing(0)
        
        button_layout.addStretch(1)
        
        self.dark_cal_button = create_utility_button("Dark Calibrate")
        self.dark_cal_button.clicked.connect(self.on_dark_calibrate)
        button_layout.addWidget(self.dark_cal_button)
        
        button_layout.addStretch(1)
        
        self.ffc_cal_button = create_utility_button("FFC Calibrate")
        self.ffc_cal_button.clicked.connect(self.on_ffc_calibrate)
        button_layout.addWidget(self.ffc_cal_button)
        
        button_layout.addStretch(1)
        
        layout.addWidget(button_container, stretch=30)
        
        return panel
    
    def create_center_panel(self) -> QWidget:
        """Create center panel with all sliders"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(0)
        
        # Title
        title_label = QLabel("Control Parameters")
        title_label.setStyleSheet("font-size: 16px; font-weight: bold; text-align: center;")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setMaximumHeight(30)
        layout.addWidget(title_label)
        
        # Sliders container - evenly spread
        slider_widget = QWidget()
        slider_layout = QVBoxLayout(slider_widget)
        slider_layout.setContentsMargins(0, 10, 0, 0)
        slider_layout.setSpacing(0)
        
        self.sliders = {}
        self.slider_labels = {}
        
        num_sliders = len(self.slider_configs)
        for i, (name, config) in enumerate(self.slider_configs.items()):
            container, slider, label = create_slider_widget(name, config)
            slider.valueChanged.connect(
                lambda value, n=name, c=config, lbl=label, s=slider:
                self.on_slider_value_changed(n, c, lbl, s, value)
            )
            slider.sliderReleased.connect(
                lambda n=name, c=config, lbl=label, s=slider:
                self.on_slider_released(n, c, lbl, s)
            )
            self.sliders[name] = slider
            self.slider_labels[name] = label
            slider_layout.addWidget(container, stretch=1)

            # Add stretch between sliders but not after the last one
            if i < num_sliders - 1:
                slider_layout.addStretch(1)
        
        layout.addWidget(slider_widget)
        
        return panel

    
    def create_right_panel(self) -> QWidget:
        """Create right panel with both images stacked vertically"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(10, 10, 10, 10)
        layout.setSpacing(10)
        
        # Image 1 (50% of height)
        img1_widget, self.img1_image_label = create_image_panel("Image 1")
        layout.addWidget(img1_widget, stretch=50)
        
        # Image 2 (50% of height)
        img2_widget, self.img2_image_label = create_image_panel("Image 2")
        layout.addWidget(img2_widget, stretch=50)
        
        return panel
    
    def setup_ros_interfaces(self):
        """Setup ROS subscriptions and service clients"""
        from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
        
        reliability_str = self.node.get_parameter('image_sub_qos_reliability').value
        reliability = (
            QoSReliabilityPolicy.RELIABLE
            if reliability_str == 'reliable'
            else QoSReliabilityPolicy.BEST_EFFORT
        )
        qos = QoSProfile(
            reliability=reliability,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1  # Only keep the latest message
        )
        
        # Image subscriptions
        img1_topic = self.node.get_parameter('image_topics.img1').value
        img2_topic = self.node.get_parameter('image_topics.img2').value
        
        # Check if topics are compressed based on topic name
        img1_is_compressed = 'compressed' in img1_topic
        img2_is_compressed = 'compressed' in img2_topic
        
        if img1_is_compressed:
            self.img1_sub = self.node.create_subscription(
                CompressedImage, img1_topic, self.img1_compressed_callback, qos
            )
        else:
            self.img1_sub = self.node.create_subscription(
                Image, img1_topic, self.img1_image_callback, qos
            )
        
        if img2_is_compressed:
            self.img2_sub = self.node.create_subscription(
                CompressedImage, img2_topic, self.img2_compressed_callback, qos
            )
        else:
            self.img2_sub = self.node.create_subscription(
                Image, img2_topic, self.img2_image_callback, qos
            )
        
        # Trigger service clients
        start_service = self.node.get_parameter('trigger_start_service').value
        stop_service = self.node.get_parameter('trigger_stop_service').value
        
        self.trigger_start_client = self.node.create_client(Trigger, start_service)
        self.trigger_stop_client = self.node.create_client(Trigger, stop_service)

        is_running_service = self.node.get_parameter('trigger_is_running_service').value
        self.trigger_is_running_client = self.node.create_client(Trigger, is_running_service)

        reload_ffc_service = self.node.get_parameter('ximea_reload_ffc_service').value
        self.ximea_reload_ffc_client = self.node.create_client(Trigger, reload_ffc_service)
        
        # Parameter clients for each controllable node
        # We'll set parameters directly using set_parameters service calls
        
        self.log(f"Subscribed to {img1_topic}")
        self.log(f"Subscribed to {img2_topic}")
        self.log(f"Configured trigger services: {start_service}, {stop_service}")
    
    def img1_compressed_callback(self, msg: CompressedImage):
        """Callback for compressed image 1"""
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.signals.image_img1.emit(cv_image)
            self._maybe_check_video_state()
        except Exception as e:
            self.node.get_logger().error(f"Error processing compressed image 1: {e}")
    
    def img2_compressed_callback(self, msg: CompressedImage):
        """Callback for compressed image 2"""
        try:
            cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.signals.image_img2.emit(cv_image)
            self._maybe_check_video_state()
            self._collect_calibration_frame(cv_image)
        except Exception as e:
            self.node.get_logger().error(f"Error processing compressed image 2: {e}")
    
    def img1_image_callback(self, msg: Image):
        """Callback for image 1"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.signals.image_img1.emit(cv_image)
            self._maybe_check_video_state()
        except Exception as e:
            self.node.get_logger().error(f"Error processing image 1: {e}")
    
    def img2_image_callback(self, msg: Image):
        """Callback for image 2"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.signals.image_img2.emit(cv_image)
            self._maybe_check_video_state()
            self._collect_calibration_frame(cv_image)
        except Exception as e:
            self.node.get_logger().error(f"Error processing image 2: {e}")
    
    def update_img1_image(self, cv_image: np.ndarray):
        """Update image 1 display"""
        self.update_image_label(self.img1_image_label, cv_image)
    
    def update_img2_image(self, cv_image: np.ndarray):
        """Update image 2 display"""
        self.update_image_label(self.img2_image_label, cv_image)
    
    def update_image_label(self, label: QLabel, cv_image: np.ndarray):
        """Convert OpenCV image to QPixmap and display"""
        height, width, channel = cv_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(cv_image.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
        
        # Scale to fit label size while maintaining aspect ratio
        # Use FastTransformation for lower latency (trade quality for speed)
        pixmap = QPixmap.fromImage(q_image)
        scaled_pixmap = pixmap.scaled(
            label.size(), Qt.KeepAspectRatio, Qt.FastTransformation
        )
        label.setPixmap(scaled_pixmap)
    
    def check_and_sync_nodes(self):
        """Check for nodes coming online and push GUI parameter values to them"""
        # Get list of all target nodes from slider configs
        target_nodes = set()
        for config in self.slider_configs.values():
            target_nodes.add(config['node'])
        
        # Check each node
        for node_name in target_nodes:
            if node_name in self.configured_nodes:
                continue  # Already configured
            
            # Try to create a client to the set_parameters service
            set_params_client = self.node.create_client(
                SetParameters,
                f'{node_name}/set_parameters'
            )
            
            # Check if service is available
            if set_params_client.wait_for_service(timeout_sec=0.1):
                self.log(f"Node {node_name} detected, syncing parameters...")
                self.sync_node_parameters(node_name)
                self.configured_nodes.add(node_name)
    
    def sync_node_parameters(self, node_name: str):
        """Push slider values to node, or query-first if force_default_params is False"""
        node_sliders = {name: config for name, config in self.slider_configs.items()
                        if config['node'] == node_name}
        if not node_sliders:
            return

        if self.node.get_parameter('force_default_params').value:
            # Force mode: push every slider's current value to the node immediately
            for name, config in node_sliders.items():
                self.log(f"Force-pushing {config['param'][0]} to {node_name}")
                self._push_slider_value_to_node(name, config)
            return

        get_params_client = self.node.create_client(
            GetParameters,
            f'{node_name}/get_parameters'
        )

        if not get_params_client.wait_for_service(timeout_sec=0.5):
            self.log(f"Warning: Cannot query params for {node_name}")
            return

        # Query the first param name for each slider (representative value)
        param_names = [config['param'][0] for config in node_sliders.values()]
        request = GetParameters.Request()
        request.names = param_names

        future = get_params_client.call_async(request)
        future.add_done_callback(
            lambda f: self._handle_node_param_query(f, node_name, node_sliders)
        )

    def _handle_node_param_query(self, future, node_name: str, node_sliders: dict):
        """Handle get_parameters response: update sliders for set params, push defaults for unset"""
        try:
            response = future.result()
            for i, (name, config) in enumerate(node_sliders.items()):
                param_value = response.values[i] if i < len(response.values) else None

                if param_value and param_value.type != ParameterType.PARAMETER_NOT_SET:
                    # Parameter already has a value on the node — update the slider to match
                    if param_value.type == ParameterType.PARAMETER_DOUBLE:
                        value = param_value.double_value
                    elif param_value.type == ParameterType.PARAMETER_INTEGER:
                        value = param_value.integer_value
                    else:
                        self._push_slider_value_to_node(name, config)
                        continue

                    value = max(config['min'], min(config['max'], value))

                    slider = self.sliders.get(name)
                    label = self.slider_labels.get(name)
                    if slider and label:
                        has_step = config.get('step') is not None
                        is_int = (isinstance(config['min'], int) and
                                  isinstance(config['max'], int) and not has_step)
                        if is_int:
                            slider.setValue(int(value))
                            label.setText(f"{name}: {int(value)}")
                        elif has_step:
                            step_position = int((value - config['min']) / config['step'])
                            slider.setValue(step_position)
                            label.setText(f"{name}: {value:.1f}")
                        else:
                            range_val = config['max'] - config['min']
                            scaled_value = int(((value - config['min']) / range_val) * 1000)
                            slider.setValue(scaled_value)
                            label.setText(f"{name}: {value:.2f}")
                        self.log(f"Read {node_name}/{config['param'][0]} = {value}, slider updated")
                else:
                    # Parameter not set on the node — push the GUI default
                    self.log(f"Param {config['param'][0]} not set on {node_name}, pushing default")
                    self._push_slider_value_to_node(name, config)
        except Exception as e:
            self.log(f"Error handling param query for {node_name}: {e}")

    def _push_slider_value_to_node(self, name: str, config: dict):
        """Push the current slider value to the target node"""
        slider = self.sliders[name]
        value = slider.value()

        has_step = config.get('step') is not None
        is_int = isinstance(config['min'], int) and isinstance(config['max'], int) and not has_step

        if is_int:
            actual_value = int(value)
        elif has_step:
            actual_value = float(config['min'] + (value * config['step']))
        else:
            actual_value = float(config['min'] + (value / 1000.0) * (config['max'] - config['min']))

        node_name = config['node']
        for param_name in config['param']:
            self.log(f"Pushing default {node_name}/{param_name} = {actual_value}")
            self.update_parameter(node_name, param_name, actual_value)

        if config.get('service'):
            QTimer.singleShot(100, lambda s=config['service'], p=config['param'][0], v=actual_value:
                              self.call_trigger_service(s, p, v))
    
    def check_recording_state(self):
        """Check if video recording is already active on startup"""
        try:
            if not self.trigger_is_running_client.wait_for_service(timeout_sec=1.0):
                self.log("Warning: Cannot check recording state - trigger service not available")
                return

            future = self.trigger_is_running_client.call_async(Trigger.Request())
            future.add_done_callback(self.check_recording_state_callback)

        except Exception as e:
            self.log(f"Error checking recording state: {e}")

    def check_recording_state_callback(self, future):
        """Handle response from recording state check"""
        try:
            response = future.result()
            if response.success:
                # Video was already running - update GUI to match
                self.recording = True
                self.start_button.setText("STOP")
                self.start_button.setStyleSheet("""
                    QPushButton {
                        background-color: #FF4444;
                        color: white;
                        font-size: 32px;
                        font-weight: bold;
                        border: 2px solid #CC0000;
                        border-radius: 10px;
                    }
                    QPushButton:hover {
                        background-color: #FF5555;
                    }
                    QPushButton:pressed {
                        background-color: #DD3333;
                    }
                """)
                self.log("Detected video already running - GUI synced")
            else:
                self.recording = False
                self.log("Checked recording state: stopped")
        except Exception as e:
            self.log(f"Error in recording state callback: {e}")

    def _maybe_check_video_state(self):
        """Throttled video state poll triggered by incoming images (at most every 1 s)"""
        now = time.monotonic()
        if now - self._last_video_state_check < 1.0:
            return
        self._last_video_state_check = now
        if not self.trigger_is_running_client.service_is_ready():
            return
        future = self.trigger_is_running_client.call_async(Trigger.Request())
        future.add_done_callback(self._video_state_poll_callback)

    def _video_state_poll_callback(self, future):
        """Update START/STOP button if video state changed externally"""
        try:
            response = future.result()
            video_running = response.success
            if video_running and not self.recording:
                self.recording = True
                self.start_button.setText("STOP")
                self.start_button.setStyleSheet("""
                    QPushButton {
                        background-color: #FF4444;
                        color: white;
                        font-size: 32px;
                        font-weight: bold;
                        border: 2px solid #CC0000;
                        border-radius: 10px;
                    }
                    QPushButton:hover {
                        background-color: #FF5555;
                    }
                    QPushButton:pressed {
                        background-color: #DD3333;
                    }
                """)
                self.log("Video started externally - GUI updated to STOP")
            elif not video_running and self.recording:
                self.recording = False
                self.reset_start_button()
                self.log("Video stopped externally - GUI updated to START")
        except Exception:
            pass  # Silently ignore transient poll errors
    
    def on_slider_value_changed(self, name: str, config: Dict[str, Any], 
                                label: QLabel, slider: QSlider, value: int):
        """Handle slider value changes while dragging (only update label, no logging)"""
        # Convert slider value to actual parameter value based on slider type
        has_step = config.get('step') is not None
        is_int = isinstance(config['min'], int) and isinstance(config['max'], int) and not has_step
        
        if is_int:
            actual_value = value
            label.setText(f"{name}: {actual_value}")
        elif has_step:
            # Step-based float (e.g., 0.1 increments)
            actual_value = config['min'] + (value * config['step'])
            label.setText(f"{name}: {actual_value:.1f}")
        else:
            # Smooth float
            range_val = config['max'] - config['min']
            actual_value = config['min'] + (value / 1000.0) * range_val
            label.setText(f"{name}: {actual_value:.2f}")
    
    def on_slider_released(self, name: str, config: Dict[str, Any],
                          label: QLabel, slider: QSlider):
        """Handle slider release (log and set parameter)"""
        # Get current slider value
        value = slider.value()
        
        # Convert slider value to actual parameter value based on slider type
        has_step = config.get('step') is not None
        is_int = isinstance(config['min'], int) and isinstance(config['max'], int) and not has_step
        
        if is_int:
            actual_value = value
        elif has_step:
            # Step-based float (e.g., 0.1 increments)
            actual_value = config['min'] + (value * config['step'])
        else:
            # Smooth float
            range_val = config['max'] - config['min']
            actual_value = config['min'] + (value / 1000.0) * range_val
        
        # Update parameter(s) on target node and log
        self.log(f"Setting {name} = {actual_value}")
        
        # Handle multiple parameters (param is now always a list)
        for param_name in config['param']:
            self.update_parameter(config['node'], param_name, actual_value)
        
        # If this parameter has an associated service (e.g., trigger services), call it
        if config.get('service'):
            self.call_trigger_service(config['service'], name, actual_value)

        # Persist the new value as the default for next session
        self._persist_slider_default(name, config, actual_value)
    
    def _persist_slider_default(self, name: str, config: dict, value):
        """Write the slider value back as the new default in gui_params.yaml."""
        config_path = self.node.get_parameter('gui_config_file_path').value
        if not config_path:
            return
        base_name = config.get('base_name')
        if not base_name:
            return
        try:
            with open(config_path, 'r') as f:
                lines = f.readlines()

            has_step = config.get('step') is not None
            is_int = (isinstance(config['min'], int) and
                      isinstance(config['max'], int) and not has_step)
            new_val = int(round(value)) if is_int else float(value)

            # Find the section block for base_name and update its `default:` line
            in_section = False
            section_indent = None
            for i, line in enumerate(lines):
                stripped = line.lstrip()
                indent = len(line) - len(stripped)
                if not in_section:
                    if stripped.startswith(f'{base_name}:'):
                        in_section = True
                        section_indent = indent
                else:
                    # A non-blank, non-comment line at same or lower indent exits the section
                    if stripped and not stripped.startswith('#') and indent <= section_indent:
                        break
                    if stripped.startswith('default:'):
                        lines[i] = ' ' * indent + f'default: {new_val}\n'
                        break

            with open(config_path, 'w') as f:
                f.writelines(lines)
        except Exception as e:
            self.node.get_logger().warn(f'Could not persist default for {base_name}: {e}')

    def update_parameter(self, node_name: str, param_name: str, value):
        """Update parameter on target node"""
        try:
            # Create a parameter client for the target node
            param_client = self.node.create_client(
                SetParameters,
                f'{node_name}/set_parameters'
            )
            
            # Wait for service to be available
            if not param_client.wait_for_service(timeout_sec=1.0):
                self.log(f"Warning: Parameter service for {node_name} not available")
                return
            
            # Create parameter value
            param = RclParameter()
            param.name = param_name
            param.value = ParameterValue()
            
            # Set type based on value type
            if isinstance(value, int):
                param.value.type = ParameterType.PARAMETER_INTEGER
                param.value.integer_value = value
            else:
                param.value.type = ParameterType.PARAMETER_DOUBLE
                param.value.double_value = float(value)
            
            # Create and send request
            request = SetParameters.Request()
            request.parameters = [param]
            
            future = param_client.call_async(request)
            future.add_done_callback(
                lambda f, pn=param_name: self.set_parameter_callback(f, pn)
            )
            
        except Exception as e:
            self.log(f"Error setting parameter: {e}")
    
    def set_parameter_callback(self, future, param_name: str):
        """Handle set parameter response"""
        try:
            response = future.result()
            if response.results and not response.results[0].successful:
                self.log(f"Warning: Failed to set {param_name}: {response.results[0].reason}")
        except Exception as e:
            self.log(f"Error in set parameter callback: {e}")
    
    def call_trigger_service(self, service_name: str, param_name: str, value: float):
        """Call a trigger service to apply parameter changes"""
        try:
            client = self.node.create_client(Trigger, service_name)
            if client.wait_for_service(timeout_sec=1.0):
                request = Trigger.Request()
                future = client.call_async(request)
                future.add_done_callback(
                    lambda f, pn=param_name, v=value: self.trigger_service_callback(f, pn, v)
                )
            else:
                self.log(f"Warning: Service {service_name} not available")
        except Exception as e:
            self.log(f"Error calling trigger service: {e}")
    
    def trigger_service_callback(self, future, param_name: str, value: float):
        """Handle trigger service response"""
        try:
            response = future.result()
            if response.success:
                self.log(f"{param_name} updated to {value}")
            else:
                self.log(f"Warning: {param_name} service call failed: {response.message}")
        except Exception as e:
            self.log(f"Error in trigger service callback: {e}")
    
    def toggle_recording(self):
        """Toggle recording state"""
        if not self.recording:
            self.start_recording()
        else:
            self.stop_recording()
    
    def start_recording(self):
        """Start video recording"""
        self.log("Starting recording...")
        
        # Change button appearance
        self.start_button.setText("STOP")
        self.start_button.setStyleSheet("""
            QPushButton {
                background-color: #FF4444;
                color: white;
                font-size: 32px;
                font-weight: bold;
                border: 2px solid #CC0000;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #FF5555;
            }
            QPushButton:pressed {
                background-color: #DD3333;
            }
        """)
        
        # Wait for service with longer timeout
        if not self.trigger_start_client.wait_for_service(timeout_sec=5.0):
            self.log("ERROR: Start video service not available")
            # Reset button
            self.reset_start_button()
            return
        
        # Call trigger service
        request = Trigger.Request()
        future = self.trigger_start_client.call_async(request)
        future.add_done_callback(self.start_recording_callback)
    
    def start_recording_callback(self, future):
        """Handle start recording service response"""
        try:
            response = future.result()
            if response.success:
                self.recording = True
                self.log("Recording started successfully")
            else:
                self.log(f"Failed to start recording: {response.message}")
                # Reset button
                self.recording = False
                self.reset_start_button()
        except Exception as e:
            self.log(f"Error calling start service: {e}")
            self.recording = False
            self.reset_start_button()
    
    def reset_start_button(self):
        """Reset start button to initial state"""
        self.start_button.setText("START")
        self.start_button.setStyleSheet("""
            QPushButton {
                background-color: #808080;
                color: white;
                font-size: 32px;
                font-weight: bold;
                border: 2px solid #404040;
                border-radius: 10px;
            }
            QPushButton:hover {
                background-color: #909090;
            }
            QPushButton:pressed {
                background-color: #707070;
            }
        """)
    
    def stop_recording(self):
        """Stop video recording"""
        self.log("Stopping recording...")
        
        # Change button appearance back
        self.reset_start_button()
        
        # Wait for service
        if not self.trigger_stop_client.wait_for_service(timeout_sec=5.0):
            self.log("ERROR: Stop video service not available")
            self.recording = False
            return
        
        # Call stop service
        request = Trigger.Request()
        future = self.trigger_stop_client.call_async(request)
        future.add_done_callback(self.stop_recording_callback)
    
    def stop_recording_callback(self, future):
        """Handle stop recording service response"""
        try:
            response = future.result()
            if response.success:
                self.recording = False
                self.log("Recording stopped successfully")
            else:
                self.log(f"Failed to stop recording: {response.message}")
                self.recording = False
        except Exception as e:
            self.log(f"Error calling stop service: {e}")
            self.recording = False
    
    def on_dark_calibrate(self):
        """Capture 5 dark frames (lens cap on XIMEA), average and save as <timestamp>_dark.tif"""
        if self._calibration_mode is not None:
            self.log("Calibration already in progress — please wait.")
            return
        self._calibration_mode = 'dark'
        self._calibration_frames = []
        self.dark_cal_button.setEnabled(False)
        self.ffc_cal_button.setEnabled(False)
        self.log("=== Dark Calibration Started ===")
        self.log("Please ensure the lens cap is on the XIMEA camera.")
        self.log("Waiting for 5 dark frames from the XIMEA camera... (0/5)")
        self._disable_ximea_ffc_then_start()

    def on_ffc_calibrate(self):
        """Capture 5 FFC frames (white board at 20-30 cm from XIMEA), average and save as <timestamp>_mid.tif"""
        if self._calibration_mode is not None:
            self.log("Calibration already in progress — please wait.")
            return
        self._calibration_mode = 'ffc'
        self._calibration_frames = []
        self.dark_cal_button.setEnabled(False)
        self.ffc_cal_button.setEnabled(False)
        self.log("=== FFC Calibration Started ===")
        self.log("Hold the white FFC calibration board 20–30 cm in front of the XIMEA camera,")
        self.log("illuminated with the same lighting used during normal operation.")
        self.log("Waiting for 5 frames from the XIMEA camera... (0/5)")
        self._disable_ximea_ffc_then_start()

    def _start_calibration_timeout(self):
        """Start a 30-second watchdog; warn the user if not enough frames arrive."""
        if self._calibration_timeout_timer is not None:
            self._calibration_timeout_timer.stop()
        self._calibration_timeout_timer = QTimer()
        self._calibration_timeout_timer.setSingleShot(True)
        self._calibration_timeout_timer.timeout.connect(self._calibration_timed_out)
        self._calibration_timeout_timer.start(30000)

    def _disable_ximea_ffc_then_start(self):
        """Disable FFC on the XIMEA node so raw uncorrected frames are collected."""
        ximea_node = self.slider_configs['Ximea Gain (dB)']['node']
        try:
            client = self.node.create_client(SetParameters, f'{ximea_node}/set_parameters')
            if not client.wait_for_service(timeout_sec=1.0):
                self.log("WARNING: XIMEA set_parameters service not available — "
                         "calibration will capture already-corrected images.")
                self._start_calibration_timeout()
                return
            param = RclParameter()
            param.name = 'enable_ffc'
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = False
            request = SetParameters.Request()
            request.parameters = [param]
            future = client.call_async(request)
            future.add_done_callback(self._ximea_ffc_disabled_callback)
        except Exception as e:
            self.log(f"WARNING: Could not disable XIMEA FFC: {e} — proceeding anyway.")
            self._start_calibration_timeout()

    def _ximea_ffc_disabled_callback(self, future):
        """Called once enable_ffc=False is confirmed; starts the actual frame collection."""
        try:
            response = future.result()
            if response.results and response.results[0].successful:
                self.log("XIMEA FFC disabled — collecting raw uncorrected frames.")
            else:
                self.log("WARNING: Could not disable FFC on XIMEA — images may be pre-corrected.")
        except Exception as e:
            self.log(f"WARNING: Error disabling XIMEA FFC: {e} — proceeding anyway.")
        self._start_calibration_timeout()

    def _reenable_ximea_ffc(self):
        """Re-enable FFC on the XIMEA node after calibration is complete."""
        ximea_node = self.slider_configs['Ximea Gain (dB)']['node']
        try:
            client = self.node.create_client(SetParameters, f'{ximea_node}/set_parameters')
            if not client.wait_for_service(timeout_sec=1.0):
                self.log("WARNING: Could not re-enable XIMEA FFC — restart the camera node.")
                self.log("=== Calibration Complete ===")
                return
            param = RclParameter()
            param.name = 'enable_ffc'
            param.value = ParameterValue()
            param.value.type = ParameterType.PARAMETER_BOOL
            param.value.bool_value = True
            request = SetParameters.Request()
            request.parameters = [param]
            future = client.call_async(request)
            future.add_done_callback(self._ximea_ffc_reenabled_callback)
        except Exception as e:
            self.log(f"WARNING: Could not re-enable XIMEA FFC: {e}")
            self.log("=== Calibration Complete ===")

    def _ximea_ffc_reenabled_callback(self, future):
        """Called once enable_ffc=True is confirmed; signals end of calibration."""
        try:
            response = future.result()
            if response.results and response.results[0].successful:
                self.log("XIMEA FFC re-enabled — normal corrected operation resumed.")
            else:
                self.log("WARNING: Could not re-enable FFC on XIMEA — restart the camera node.")
        except Exception as e:
            self.log(f"WARNING: Error re-enabling XIMEA FFC: {e}")
        self.log("=== Calibration Complete ===")

    def _calibration_timed_out(self):
        """Called when calibration watchdog fires before 5 frames were collected."""
        if self._calibration_mode is None:
            return
        collected = len(self._calibration_frames)
        self.log(f"WARNING: Calibration timed out — only {collected}/5 frames received.")
        self.log("Ensure the XIMEA camera is publishing images and the hardware trigger is running.")
        self._calibration_mode = None
        self._calibration_frames = []
        self._calibration_timeout_timer = None
        self.dark_cal_button.setEnabled(True)
        self.ffc_cal_button.setEnabled(True)

    def _collect_calibration_frame(self, cv_image: np.ndarray):
        """Collect a single calibration frame from img2. Triggers save when 5 frames are gathered."""
        if self._calibration_mode is None:
            return
        if len(self._calibration_frames) >= 5:
            return  # already collecting the save, ignore extra frames

        # Convert to grayscale (ximea publishes mono8; bgr channels are equal)
        if len(cv_image.shape) == 3:
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        else:
            gray = cv_image

        self._calibration_frames.append(gray.astype(np.float32))
        count = len(self._calibration_frames)
        self.log(f"Frame {count}/5 captured")

        if count == 5:
            # Schedule finalization outside this callback to avoid blocking the ROS spin
            QTimer.singleShot(0, self._finalize_calibration)

    def _finalize_calibration(self):
        """Average the 5 collected frames and save as a grayscale TIFF."""
        if not self._calibration_frames:
            return

        if self._calibration_timeout_timer is not None:
            self._calibration_timeout_timer.stop()
            self._calibration_timeout_timer = None

        mode = self._calibration_mode
        frames = self._calibration_frames
        self._calibration_mode = None
        self._calibration_frames = []

        # Pixel-wise average of all 5 frames
        averaged = np.mean(np.stack(frames, axis=0), axis=0).astype(np.uint8)

        ffc_dir = os.path.expanduser(self.node.get_parameter('ffc_dir').value)
        os.makedirs(ffc_dir, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        suffix = '_dark.tif' if mode == 'dark' else '_mid.tif'
        filename = f"{timestamp}{suffix}"
        save_path = os.path.join(ffc_dir, filename)

        success = cv2.imwrite(save_path, averaged)

        if success:
            label = "Dark" if mode == 'dark' else "FFC (mid)"
            self.log(f"{label} calibration complete — 5 frames averaged.")
            self.log(f"Saved to: {save_path}")
            self.log("Reloading FFC calibration on XIMEA camera node...")
            self._reload_ximea_ffc()
        else:
            self.log(f"ERROR: Failed to write calibration image to: {save_path}")
            self.log("Check that the ffc_dir path is writable.")
            self._reenable_ximea_ffc()

        self.dark_cal_button.setEnabled(True)
        self.ffc_cal_button.setEnabled(True)

    def _reload_ximea_ffc(self):
        """Ask the XIMEA camera node to reload FFC calibration from ffc_dir."""
        if not self.ximea_reload_ffc_client.service_is_ready():
            self.log("WARNING: XIMEA reload_ffc service not available — restart the camera node to apply new calibration.")
            self._reenable_ximea_ffc()
            return
        future = self.ximea_reload_ffc_client.call_async(Trigger.Request())
        future.add_done_callback(self._reload_ximea_ffc_callback)

    def _reload_ximea_ffc_callback(self, future):
        """Handle response from XIMEA reload_ffc service, then re-enable FFC."""
        try:
            response = future.result()
            if response.success:
                self.log(f"XIMEA FFC reloaded: {response.message}")
            else:
                self.log(f"WARNING: XIMEA FFC reload failed: {response.message}")
        except Exception as e:
            self.log(f"ERROR calling XIMEA reload_ffc service: {e}")
        # Re-enable FFC now that the new calibration is loaded
        self._reenable_ximea_ffc()
    
    def toggle_bag_recording(self):
        """Toggle ROS2 bag recording"""
        if not self.bag_recording:
            self.start_bag_recording()
        else:
            self.stop_bag_recording()
    
    def start_bag_recording(self):
        """Start ROS2 bag recording"""
        try:
            # Get recording parameters
            topics = self.node.get_parameter('recording.topics').value
            storage_path = os.path.expanduser(self.node.get_parameter('recording.storage_path').value)
            storage_id = self.node.get_parameter('recording.storage_id').value
            
            # Create storage directory if it doesn't exist
            os.makedirs(storage_path, exist_ok=True)
            self.log(f"Storage directory: {storage_path}")
            
            # Generate bag file name with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            bag_name = f"multicam_rig_{timestamp}"
            bag_path = os.path.join(storage_path, bag_name)
            
            # Store the bag path for later
            self.current_bag_path = bag_path
            
            # Build ros2 bag command
            cmd = [
                'ros2', 'bag', 'record',
                '-o', bag_path,
                '-s', storage_id,
            ]
            
            # Add topics
            cmd.extend(topics)
            
            # Log the command being executed
            self.log(f"Executing: {' '.join(cmd)}")
            
            # Start recording process
            self.bag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # Check if process started successfully
            import time
            time.sleep(0.5)  # Give it a moment to start
            if self.bag_process.poll() is not None:
                # Process already terminated
                stdout, stderr = self.bag_process.communicate()
                self.log(f"ERROR: Bag recording failed to start!")
                if stdout:
                    self.log(f"stdout: {stdout}")
                if stderr:
                    self.log(f"stderr: {stderr}")
                self.bag_recording = False
                self.bag_process = None
                self.current_bag_path = None
                return
            
            self.bag_recording = True
            
            # Update button appearance
            self.record_button.setText("RECORDING")
            self.record_button.setStyleSheet("""
                QPushButton {
                    background-color: #FF4444;
                    color: white;
                    font-size: 28px;
                    font-weight: bold;
                    border: 2px solid #CC0000;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background-color: #FF5555;
                }
                QPushButton:pressed {
                    background-color: #DD3333;
                }
            """)
            
            self.log(f"Started bag recording: {bag_name}")
            self.log(f"Recording topics: {', '.join(topics)}")
            
        except Exception as e:
            self.log(f"Error starting bag recording: {e}")
            import traceback
            self.log(f"Traceback: {traceback.format_exc()}")
            self.bag_recording = False
    
    def stop_bag_recording(self):
        """Stop ROS2 bag recording"""
        try:
            if self.bag_process:
                self.log(f"Stopping bag process (PID: {self.bag_process.pid})...")
                
                # Send SIGINT to gracefully stop recording
                self.bag_process.send_signal(signal.SIGINT)
                
                # Wait for process to finish (with timeout)
                try:
                    self.bag_process.wait(timeout=10.0)
                    self.log("Bag process terminated successfully")
                except subprocess.TimeoutExpired:
                    self.log("Warning: Bag recording process did not stop gracefully, forcing...")
                    self.bag_process.kill()
                    self.bag_process.wait()
                
                # Get any output from the process
                try:
                    stdout, stderr = self.bag_process.communicate(timeout=1.0)
                    if stdout:
                        self.log(f"Bag stdout: {stdout[:200]}")
                    if stderr:
                        self.log(f"Bag stderr: {stderr[:200]}")
                except:
                    pass
                
                self.bag_process = None
            
            self.bag_recording = False
            
            # Reset button appearance
            self.record_button.setText("RECORD")
            self.record_button.setStyleSheet("""
                QPushButton {
                    background-color: #808080;
                    color: white;
                    font-size: 28px;
                    font-weight: bold;
                    border: 2px solid #404040;
                    border-radius: 10px;
                }
                QPushButton:hover {
                    background-color: #909090;
                }
                QPushButton:pressed {
                    background-color: #707070;
                }
            """)
            
            self.log("Stopped bag recording")
            
            # Display bag path and size information
            if self.current_bag_path:
                self.log(f"Checking for bag at: {self.current_bag_path}")
                
                if os.path.exists(self.current_bag_path):
                    # Calculate total size of bag directory
                    total_size = 0
                    file_count = 0
                    for dirpath, dirnames, filenames in os.walk(self.current_bag_path):
                        for filename in filenames:
                            filepath = os.path.join(dirpath, filename)
                            if os.path.exists(filepath):
                                total_size += os.path.getsize(filepath)
                                file_count += 1
                    
                    # Format size in human-readable format
                    if total_size < 1024:
                        size_str = f"{total_size} B"
                    elif total_size < 1024 * 1024:
                        size_str = f"{total_size / 1024:.2f} KB"
                    elif total_size < 1024 * 1024 * 1024:
                        size_str = f"{total_size / (1024 * 1024):.2f} MB"
                    else:
                        size_str = f"{total_size / (1024 * 1024 * 1024):.2f} GB"
                    
                    self.log(f"Bag saved to: {self.current_bag_path}")
                    self.log(f"Bag size: {size_str} ({file_count} files)")
                else:
                    self.log(f"WARNING: Bag directory not found at {self.current_bag_path}")
                    # List parent directory contents
                    parent_dir = os.path.dirname(self.current_bag_path)
                    if os.path.exists(parent_dir):
                        contents = os.listdir(parent_dir)
                        self.log(f"Parent directory contents: {contents[:10]}")
                
                self.current_bag_path = None
            else:
                self.log("No bag path was stored")
            
        except Exception as e:
            self.log(f"Error stopping bag recording: {e}")
            import traceback
            self.log(f"Traceback: {traceback.format_exc()}")
            self.bag_recording = False
    
    def log(self, message: str):
        """Add message to log output"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.signals.log_message.emit(f"[{timestamp}] {message}")
    
    def append_log(self, message: str):
        """Append message to log widget (called from signal)"""
        self.log_output.append(message)
        # Auto-scroll to bottom
        self.log_output.verticalScrollBar().setValue(
            self.log_output.verticalScrollBar().maximum()
        )
    
    def spin_ros(self):
        """Spin ROS node to process callbacks"""
        rclpy.spin_once(self.node, timeout_sec=0)
    
    def closeEvent(self, event):
        """Handle window close event"""
        self.ros_timer.stop()
        self.node_sync_timer.stop()
        if self.recording:
            self.stop_recording()
        if self.bag_recording:
            self.stop_bag_recording()
        event.accept()


def main(args=None):
    """Main entry point"""
    # Initialize ROS
    rclpy.init(args=args)
    
    # Create ROS node
    node = Node('camera_rig_gui_node')
    
    # Create Qt application
    app = QApplication(sys.argv)
    
    # Create and show GUI
    gui = CameraRigGUI(node)
    gui.show()
    
    # Setup signal handlers to close GUI on SIGINT/SIGTERM
    def signal_handler(sig, frame):
        """Handle termination signals"""
        print("\nReceived termination signal, shutting down...")
        gui.close()
        app.quit()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    # Allow Python to handle signals during Qt event loop
    timer = QTimer()
    timer.start(500)  # Check for signals every 500ms
    timer.timeout.connect(lambda: None)
    
    # Run application
    exit_code = app.exec_()
    
    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
