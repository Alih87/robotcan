import sys

from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QComboBox,
    QSpinBox,
    QCheckBox,
    QLineEdit,
    QTextEdit,
    QMessageBox,
)

import rclpy
from rclpy.node import Node

from robotcan_interfaces.srv import SendDriveCmd
from robotcan_interfaces.srv import SendValveCmd
from robotcan_interfaces.srv import SendDutyCmd


class RobotCanClient(Node):
    def __init__(self):
        super().__init__('robotcan_gui_client')

        self.drive_client = self.create_client(SendDriveCmd, 'send_drive_cmd')
        self.valve_client = self.create_client(SendValveCmd, 'send_valve_cmd')
        self.duty_client = self.create_client(SendDutyCmd, 'send_duty_cmd')

    def call_drive_cmd(
        self,
        mode,
        direction,
        speed,
        steering_angle_deg,
        arm_left_open=False,
        arm_left_close=False,
        arm_right_open=False,
        arm_right_close=False,
        water_main_open=False,
        water_main_close=False,
        emergency_stop=False,
    ):
        if not self.drive_client.wait_for_service(timeout_sec=1.0):
            return False, 'Drive service not available.'

        req = SendDriveCmd.Request()
        req.mode = mode
        req.direction = direction
        req.speed = speed
        req.steering_angle_deg = steering_angle_deg

        req.arm_left_open = arm_left_open
        req.arm_left_close = arm_left_close
        req.arm_right_open = arm_right_open
        req.arm_right_close = arm_right_close
        req.water_main_open = water_main_open
        req.water_main_close = water_main_close
        req.emergency_stop = emergency_stop

        future = self.drive_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is None:
            return False, 'Drive service call failed.'

        return future.result().success, future.result().message

    def call_valve_cmd(self, valves_on, valves_off, clear_previous_state):
        if not self.valve_client.wait_for_service(timeout_sec=1.0):
            return False, 'Valve service not available.'

        req = SendValveCmd.Request()
        req.valves_on = valves_on
        req.valves_off = valves_off
        req.clear_previous_state = clear_previous_state

        future = self.valve_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is None:
            return False, 'Valve service call failed.'

        return future.result().success, future.result().message

    def call_duty_cmd(self, duty_groups):
        if not self.duty_client.wait_for_service(timeout_sec=1.0):
            return False, 'Duty service not available.'

        req = SendDutyCmd.Request()
        req.duty_groups = duty_groups

        future = self.duty_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.result() is None:
            return False, 'Duty service call failed.'

        return future.result().success, future.result().message


class RobotCanGui(QWidget):
    def __init__(self, ros_node):
        super().__init__()

        self.ros_node = ros_node

        self.setWindowTitle('Robot CAN GUI')
        self.resize(500, 500)

        layout = QVBoxLayout()

        # Drive controls
        layout.addWidget(QLabel('Drive Command'))

        self.mode_box = QComboBox()
        self.mode_box.addItems(['D', 'W'])

        self.direction_box = QComboBox()
        self.direction_box.addItems(['F', 'R', 'S'])

        self.speed_box = QSpinBox()
        self.speed_box.setRange(0, 3)
        self.speed_box.setValue(1)

        self.steering_box = QSpinBox()
        self.steering_box.setRange(-45, 45)
        self.steering_box.setValue(0)

        drive_row_1 = QHBoxLayout()
        drive_row_1.addWidget(QLabel('Mode'))
        drive_row_1.addWidget(self.mode_box)
        drive_row_1.addWidget(QLabel('Direction'))
        drive_row_1.addWidget(self.direction_box)
        layout.addLayout(drive_row_1)

        drive_row_2 = QHBoxLayout()
        drive_row_2.addWidget(QLabel('Speed'))
        drive_row_2.addWidget(self.speed_box)
        drive_row_2.addWidget(QLabel('Steering'))
        drive_row_2.addWidget(self.steering_box)
        layout.addLayout(drive_row_2)

        self.water_main_open_check = QCheckBox('Water Main Open')
        self.water_main_close_check = QCheckBox('Water Main Close')
        self.emergency_stop_check = QCheckBox('Emergency Stop')

        layout.addWidget(self.water_main_open_check)
        layout.addWidget(self.water_main_close_check)
        layout.addWidget(self.emergency_stop_check)

        self.send_drive_button = QPushButton('Send Drive Command')
        self.send_drive_button.clicked.connect(self.send_drive_command)
        layout.addWidget(self.send_drive_button)

        # Valve controls
        layout.addWidget(QLabel('Valve Command'))

        self.valves_on_edit = QLineEdit()
        self.valves_on_edit.setPlaceholderText('Valves ON, example: 1,2,5')

        self.valves_off_edit = QLineEdit()
        self.valves_off_edit.setPlaceholderText('Valves OFF, example: 3,4')

        self.clear_previous_check = QCheckBox('Clear previous valve state before command')
        self.clear_previous_check.setChecked(True)

        layout.addWidget(self.valves_on_edit)
        layout.addWidget(self.valves_off_edit)
        layout.addWidget(self.clear_previous_check)

        self.send_valve_button = QPushButton('Send Valve Command')
        self.send_valve_button.clicked.connect(self.send_valve_command)
        layout.addWidget(self.send_valve_button)

        # Duty controls
        layout.addWidget(QLabel('Duty Command'))

        self.duty_edit = QLineEdit()
        self.duty_edit.setPlaceholderText('8 duty values, example: 10,0,0,0,0,0,0,0')
        layout.addWidget(self.duty_edit)

        self.send_duty_button = QPushButton('Send Duty Command')
        self.send_duty_button.clicked.connect(self.send_duty_command)
        layout.addWidget(self.send_duty_button)

        # Log
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        layout.addWidget(self.log_box)

        self.setLayout(layout)

    def parse_int_list(self, text):
        text = text.strip()

        if not text:
            return []

        values = []

        for item in text.split(','):
            item = item.strip()
            if not item:
                continue
            values.append(int(item))

        return values

    def log(self, text):
        self.log_box.append(text)

    def show_result(self, success, message):
        status = 'OK' if success else 'FAILED'
        self.log(f'[{status}] {message}')
        
    def speed_to_text(self, speed):
	    mapping = {
			0: "Stop",
			1: "Low",
			2: "Middle",
			3: "High",
		}
		return mapping.get(speed, "Unknown")


    def direction_to_text(self, direction):
		mapping = {
			"F": "Forward",
			"R": "Reverse",
			"S": "Stop",
		}
		return mapping.get(direction, "Unknown")


    def mode_to_text(self, mode):
		mapping = {
			"D": "Driving",
			"W": "Work",
		}
		return mapping.get(mode, "Unknown")


    def build_drive_flags_text(
		self,
		arm_left_open=False,
		arm_left_close=False,
		arm_right_open=False,
		arm_right_close=False,
		water_main_open=False,
		water_main_close=False,
		emergency_stop=False,
	):
		flags = []

		if arm_left_open:
			flags.append("arm_left_open")
		if arm_left_close:
			flags.append("arm_left_close")
		if arm_right_open:
			flags.append("arm_right_open")
		if arm_right_close:
			flags.append("arm_right_close")
		if water_main_open:
			flags.append("water_main_open")
		if water_main_close:
			flags.append("water_main_close")
		if emergency_stop:
			flags.append("emergency_stop")

		if not flags:
			return "none"

		return ", ".join(flags)


    def decode_drive_command_text(
		self,
		mode,
		direction,
		speed,
		steering_angle_deg,
		arm_left_open=False,
		arm_left_close=False,
		arm_right_open=False,
		arm_right_close=False,
		water_main_open=False,
		water_main_close=False,
		emergency_stop=False,
	):
		return (
			"Decoded Drive Command\n"
			f"  CAN ID: 0x101\n"
			f"  Mode: {mode} ({self.mode_to_text(mode)})\n"
			f"  Direction: {direction} ({self.direction_to_text(direction)})\n"
			f"  Speed: {speed} ({self.speed_to_text(speed)})\n"
			f"  Steering angle: {steering_angle_deg} deg\n"
			f"  Flags: {self.build_drive_flags_text(arm_left_open, arm_left_close, arm_right_open, arm_right_close, water_main_open, water_main_close, emergency_stop)}"
		)


    def decode_valve_command_text(self, valves_on, valves_off, clear_previous_state):
		valve_mask = 0

		if not clear_previous_state:
			# GUI does not know the node's internal previous mask.
			# This is only the mask contribution from this GUI command.
			pass

		for valve in valves_on:
			if 1 <= valve <= 32:
				valve_mask |= 1 << (valve - 1)

		for valve in valves_off:
			if 1 <= valve <= 32:
				valve_mask &= ~(1 << (valve - 1))

		return (
			"Decoded Valve Command\n"
			f"  CAN ID: 0x102\n"
			f"  Valves ON: {valves_on if valves_on else 'none'}\n"
			f"  Valves OFF: {valves_off if valves_off else 'none'}\n"
			f"  Clear previous state: {clear_previous_state}\n"
			f"  Command mask from GUI input: 0x{valve_mask:08X}"
		)


    def decode_duty_command_text(self, duty_groups):
		lines = [
			"Decoded Duty Command",
			"  CAN ID: 0x103",
		]

		for i, duty in enumerate(duty_groups):
			first_valve = i * 4 + 1
			last_valve = first_valve + 3
			lines.append(
				f"  Group {i + 1}, valves {first_valve}-{last_valve}: duty {duty}"
			)

		return "\n".join(lines)

    def send_drive_command(self):
		mode = self.mode_box.currentText()
		direction = self.direction_box.currentText()
		speed = self.speed_box.value()
		steering_angle_deg = self.steering_box.value()

		water_main_open = self.water_main_open_check.isChecked()
		water_main_close = self.water_main_close_check.isChecked()
		emergency_stop = self.emergency_stop_check.isChecked()

		self.log(
			self.decode_drive_command_text(
				mode=mode,
				direction=direction,
				speed=speed,
				steering_angle_deg=steering_angle_deg,
				water_main_open=water_main_open,
				water_main_close=water_main_close,
				emergency_stop=emergency_stop,
			)
		)

		success, message = self.ros_node.call_drive_cmd(
			mode=mode,
			direction=direction,
			speed=speed,
			steering_angle_deg=steering_angle_deg,
			water_main_open=water_main_open,
			water_main_close=water_main_close,
			emergency_stop=emergency_stop,
		)

		self.show_result(success, message)

    def send_valve_command(self):
		try:
			valves_on = self.parse_int_list(self.valves_on_edit.text())
			valves_off = self.parse_int_list(self.valves_off_edit.text())
		except ValueError:
			QMessageBox.warning(self, 'Input Error', 'Valve numbers must be integers.')
			return

		clear_previous_state = self.clear_previous_check.isChecked()

		self.log(
			self.decode_valve_command_text(
				valves_on=valves_on,
				valves_off=valves_off,
				clear_previous_state=clear_previous_state,
			)
		)

		success, message = self.ros_node.call_valve_cmd(
			valves_on=valves_on,
			valves_off=valves_off,
			clear_previous_state=clear_previous_state,
		)

		self.show_result(success, message)

    def send_duty_command(self):
		try:
			duty_groups = self.parse_int_list(self.duty_edit.text())
		except ValueError:
			QMessageBox.warning(self, 'Input Error', 'Duty values must be integers.')
			return

		if len(duty_groups) != 8:
			QMessageBox.warning(self, 'Input Error', 'Please enter exactly 8 duty values.')
			return

		self.log(self.decode_duty_command_text(duty_groups))

		success, message = self.ros_node.call_duty_cmd(duty_groups)
		self.show_result(success, message)all_duty_cmd(duty_groups)
			self.show_result(success, message)


def main(args=None):
    rclpy.init(args=args)

    ros_node = RobotCanClient()

    app = QApplication(sys.argv)
    window = RobotCanGui(ros_node)
    window.show()

    exit_code = app.exec_()

    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
