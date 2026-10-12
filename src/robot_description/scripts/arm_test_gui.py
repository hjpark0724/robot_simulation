#!/usr/bin/env python3
"""Qt control panel for exercising the Gazebo arm and detachable gripper target."""

import sys

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (
    QApplication,
    QDoubleSpinBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSlider,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64


ARM_LIMIT = 1.5708
ARM_HOME = (0.0, 1.20, -1.45, -1.32, 0.0)


class ArmTestGui(QWidget):
    """Publish the command topics used by sim.launch.py without extra bridges."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('N1 Gazebo Arm Test')
        self.setMinimumWidth(560)

        self.node = Node('arm_test_gui')
        self.arm_publishers = [
            self.node.create_publisher(Float64, f'/arm{index}/cmd_pos', 10)
            for index in range(1, 6)
        ]
        self.gripper_publisher = self.node.create_publisher(
            Float64, '/gripper/cmd_pos', 10)
        self.attach_publisher = self.node.create_publisher(
            Empty, '/gripper/attach_target', 10)
        self.detach_publisher = self.node.create_publisher(
            Empty, '/gripper/detach_target', 10)

        self.arm_controls = []
        self.gripper_control = None
        self.status = QLabel('Ready: connect this GUI after sim.launch.py is running.')
        self._build_ui()

        self.spin_timer = QTimer(self)
        self.spin_timer.timeout.connect(self._spin_ros)
        self.spin_timer.start(20)

    @staticmethod
    def _new_spinbox(minimum, maximum, value):
        control = QDoubleSpinBox()
        control.setRange(minimum, maximum)
        control.setDecimals(4)
        control.setSingleStep(0.05)
        control.setValue(value)
        control.setSuffix(' rad')
        return control

    def _build_joint_row(self, layout, label, minimum, maximum, value, callback):
        slider = QSlider(Qt.Horizontal)
        slider.setRange(round(minimum * 10000), round(maximum * 10000))
        slider.setValue(round(value * 10000))
        spinbox = self._new_spinbox(minimum, maximum, value)

        def slider_changed(raw_value):
            radians = raw_value / 10000.0
            spinbox.blockSignals(True)
            spinbox.setValue(radians)
            spinbox.blockSignals(False)
            callback(radians)

        def spinbox_changed(radians):
            slider.blockSignals(True)
            slider.setValue(round(radians * 10000))
            slider.blockSignals(False)
            callback(radians)

        slider.valueChanged.connect(slider_changed)
        spinbox.valueChanged.connect(spinbox_changed)
        layout.addRow(label, self._row_widget(slider, spinbox))
        return slider, spinbox

    @staticmethod
    def _row_widget(slider, spinbox):
        row = QWidget()
        row_layout = QHBoxLayout(row)
        row_layout.setContentsMargins(0, 0, 0, 0)
        row_layout.addWidget(slider, 1)
        row_layout.addWidget(spinbox)
        return row

    def _build_ui(self):
        root = QVBoxLayout(self)
        root.addWidget(QLabel(
            'Joint commands are published as Float64 radians to the active Gazebo bridge.'))

        arm_box = QGroupBox('Arm joints')
        arm_form = QFormLayout(arm_box)
        for index, home in enumerate(ARM_HOME, start=1):
            controls = self._build_joint_row(
                arm_form, f'Arm {index}', -ARM_LIMIT, ARM_LIMIT, home,
                lambda value, joint=index: self._publish_arm(joint, value))
            self.arm_controls.append(controls)
        root.addWidget(arm_box)

        gripper_box = QGroupBox('Gripper')
        gripper_form = QFormLayout(gripper_box)
        self.gripper_control = self._build_joint_row(
            gripper_form, 'Closure', -0.95, 0.0, 0.0, self._publish_gripper)
        root.addWidget(gripper_box)

        actions = QGridLayout()
        home = QPushButton('Home pose')
        home.clicked.connect(self._home)
        open_gripper = QPushButton('Open')
        open_gripper.clicked.connect(lambda: self._set_gripper(0.0))
        close_gripper = QPushButton('Close')
        close_gripper.clicked.connect(lambda: self._set_gripper(-0.70))
        attach = QPushButton('Attach target')
        attach.clicked.connect(self._attach)
        detach = QPushButton('Detach target')
        detach.clicked.connect(self._detach)
        actions.addWidget(home, 0, 0)
        actions.addWidget(open_gripper, 0, 1)
        actions.addWidget(close_gripper, 0, 2)
        actions.addWidget(attach, 1, 0, 1, 2)
        actions.addWidget(detach, 1, 2)
        root.addLayout(actions)
        root.addWidget(self.status)

    def _publish_arm(self, joint, value):
        message = Float64()
        message.data = value
        self.arm_publishers[joint - 1].publish(message)
        self.status.setText(f'Arm {joint}: {value:.4f} rad')

    def _publish_gripper(self, value):
        message = Float64()
        message.data = value
        self.gripper_publisher.publish(message)
        self.status.setText(f'Gripper: {value:.4f} rad')

    def _home(self):
        for controls, value in zip(self.arm_controls, ARM_HOME):
            controls[1].setValue(value)
        self.status.setText('Home pose commanded.')

    def _set_gripper(self, value):
        self.gripper_control[1].setValue(value)

    def _attach(self):
        self.attach_publisher.publish(Empty())
        self.status.setText('Attach request published.')

    def _detach(self):
        self.detach_publisher.publish(Empty())
        self.status.setText('Detach request published.')

    def _spin_ros(self):
        # SIGINT / SIGTERM can shut down rclpy before Qt processes its final
        # timer event (for example when the launch system exits).
        if rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.0)
            except RuntimeError:
                pass

    def closeEvent(self, event):
        self.spin_timer.stop()
        self.node.destroy_node()
        event.accept()


def main():
    rclpy.init()
    app = QApplication(sys.argv)
    gui = ArmTestGui()
    gui.show()
    result = app.exec_()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(result)


if __name__ == '__main__':
    main()
