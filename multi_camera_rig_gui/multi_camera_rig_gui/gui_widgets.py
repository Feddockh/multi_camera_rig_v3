#!/usr/bin/env python3
"""GUI widget helper functions for the camera rig GUI"""

from typing import Dict, Any, Tuple

from PyQt5.QtWidgets import (
    QPushButton, QSlider, QLabel, QWidget,
    QVBoxLayout, QHBoxLayout, QSizePolicy,
)
from PyQt5.QtCore import Qt


_GREY_BUTTON_STYLE = """
    QPushButton {{
        background-color: {bg};
        color: white;
        font-size: {font_size}px;
        font-weight: bold;
        border: 2px solid {border};
        border-radius: 10px;
    }}
    QPushButton:hover {{
        background-color: {hover};
    }}
    QPushButton:pressed {{
        background-color: {press};
    }}
"""

_BLUE_BUTTON_STYLE = """
    QPushButton {
        background-color: #4A90E2;
        color: white;
        font-size: 16px;
        font-weight: bold;
        border: 2px solid #2E5F8F;
        border-radius: 5px;
    }
    QPushButton:hover {
        background-color: #5AA0F2;
    }
    QPushButton:pressed {
        background-color: #3A80D2;
    }
"""

_SLIDER_STYLE = """
    QSlider::groove:horizontal {
        border: 1px solid #999999;
        height: 12px;
        background: white;
        margin: 2px 0;
        border-radius: 6px;
    }
    QSlider::handle:horizontal {
        background: #5555FF;
        border: 2px solid #3333CC;
        width: 60px;
        height: 60px;
        margin: -25px 0;
        border-radius: 30px;
    }
    QSlider::handle:horizontal:hover {
        background: #6666FF;
        border: 2px solid #4444DD;
    }
"""


def create_action_button(text: str, min_height: int = 60, font_size: int = 28) -> QPushButton:
    """Create a large grey action button (e.g. START/STOP, RECORD)."""
    btn = QPushButton(text)
    btn.setMinimumHeight(min_height)
    btn.setStyleSheet(_GREY_BUTTON_STYLE.format(
        bg='#808080', border='#404040', hover='#909090', press='#707070', font_size=font_size
    ))
    return btn


def create_utility_button(text: str, min_height: int = 80) -> QPushButton:
    """Create a blue utility button (e.g. calibration actions)."""
    btn = QPushButton(text)
    btn.setMinimumHeight(min_height)
    btn.setStyleSheet(_BLUE_BUTTON_STYLE)
    return btn


def create_image_panel(title: str) -> Tuple[QWidget, QLabel]:
    """Create an image display panel. Returns (panel_widget, image_label)."""
    widget = QWidget()
    layout = QVBoxLayout(widget)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(2)

    title_label = QLabel(title)
    title_label.setStyleSheet("font-size: 14px; font-weight: bold;")
    title_label.setAlignment(Qt.AlignCenter)
    title_label.setMaximumHeight(25)
    layout.addWidget(title_label, stretch=0)

    image_label = QLabel()
    image_label.setStyleSheet("""
        QLabel {
            background-color: black;
            border: 2px solid #808080;
            border-radius: 5px;
        }
    """)
    image_label.setAlignment(Qt.AlignCenter)
    image_label.setScaledContents(False)
    image_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)
    image_label.setMinimumSize(1, 1)
    layout.addWidget(image_label, stretch=1)

    return widget, image_label


def create_slider_widget(name: str, config: Dict[str, Any]) -> Tuple[QWidget, QSlider, QLabel]:
    """
    Create a labeled slider widget. Signal connections are left to the caller.
    Returns (container_widget, slider, label).
    """
    container = QWidget()
    layout = QVBoxLayout(container)
    layout.setContentsMargins(0, 0, 0, 0)
    layout.setSpacing(1)

    has_step = config.get('step') is not None
    is_int = isinstance(config['min'], int) and isinstance(config['max'], int) and not has_step

    if is_int:
        label = QLabel(f"{name}: {int(config['default'])}")
    else:
        label = QLabel(f"{name}: {config['default']:.2f}")
    label.setStyleSheet("font-size: 11px; font-weight: bold;")
    label.setMaximumHeight(20)
    layout.addWidget(label)

    slider = QSlider(Qt.Horizontal)
    slider.setMinimumHeight(40)

    if is_int:
        slider.setMinimum(int(config['min']))
        slider.setMaximum(int(config['max']))
        slider.setValue(int(config['default']))
        slider.setTickInterval(max(1, (config['max'] - config['min']) // 10))
    elif has_step:
        step = config['step']
        num_steps = int((config['max'] - config['min']) / step)
        slider.setMinimum(0)
        slider.setMaximum(num_steps)
        default_step = int((config['default'] - config['min']) / step)
        slider.setValue(default_step)
        slider.setTickInterval(max(1, num_steps // 10))
    else:
        slider.setMinimum(0)
        slider.setMaximum(1000)
        range_val = config['max'] - config['min']
        scaled_default = int(((config['default'] - config['min']) / range_val) * 1000)
        slider.setValue(scaled_default)
        slider.setTickInterval(100)

    slider.setTickPosition(QSlider.TicksBelow)
    slider.setStyleSheet(_SLIDER_STYLE)
    layout.addWidget(slider)

    # Min/Max labels
    minmax_layout = QHBoxLayout()
    minmax_layout.setContentsMargins(0, 0, 0, 0)
    min_label = QLabel(f"{config['min']}")
    min_label.setStyleSheet("font-size: 9px; color: #666666;")
    min_label.setMaximumHeight(15)
    max_label = QLabel(f"{config['max']}")
    max_label.setStyleSheet("font-size: 9px; color: #666666;")
    max_label.setMaximumHeight(15)
    minmax_layout.addWidget(min_label)
    minmax_layout.addStretch()
    minmax_layout.addWidget(max_label)
    layout.addLayout(minmax_layout)

    return container, slider, label
