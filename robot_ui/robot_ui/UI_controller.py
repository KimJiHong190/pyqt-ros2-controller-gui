import sys
import rclpy
from rclpy.node import Node
from PyQt5.QtCore import pyqtSignal, Qt, QTimer
from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit
from std_msgs.msg import String, Float32, Float32MultiArray
from .submodule.UI_window import Ui_MainWindow
from geometry_msgs.msg import PoseStamped
from PyQt5 import QtCore, QtGui, QtWidgets
from robot_interfaces.msg import BridgeStringMultiArray, BridgeBoolMultiArray, BridgeFloat32MultiArray, BridgeJointStateWithAccel

class UIController(Node, QMainWindow):
    log_signal = pyqtSignal(str)

    def __init__(self):
        super().__init__('ui_controller')
        QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.wheel_velocity_command = 0.1

        # Initialize ROS publishers
        self.robot_activation_pub = self.create_publisher(BridgeBoolMultiArray, 'ui_setup_is_robot_activation', 10)
        self.robot2_activation_pub = self.create_publisher(BridgeBoolMultiArray, 'ui_setup_is_robot2_activation', 10)
        self.joint_activation_pub = self.create_publisher(BridgeBoolMultiArray, 'ui_setup_is_joint_activation', 10)
        self.wheel_activation_pub = self.create_publisher(BridgeBoolMultiArray, 'ui_setup_is_wheel_activation', 10) #wheel_control_button
        
        self.wheel_FL_activation_pub = self.create_publisher(BridgeBoolMultiArray, 'ui_setup_is_wheel_FL_activation', 10)
        self.wheel_FR_activation_pub = self.create_publisher(BridgeBoolMultiArray, 'ui_setup_is_wheel_FR_activation', 10)
        self.wheel_RL_activation_pub = self.create_publisher(BridgeBoolMultiArray, 'ui_setup_is_wheel_RL_activation', 10)
        self.wheel_RR_activation_pub = self.create_publisher(BridgeBoolMultiArray, 'ui_setup_is_wheel_RR_activation', 10)
        
        
        
        self.calibration_publisher = self.create_publisher(String, 'calibration_topic', 10)
        
        self.robot2_command_publisher = self.create_publisher(String, 'robot2_control_command', 10)
        self.robot1_command_publisher = self.create_publisher(String, 'robot1_control_command', 10)
        
        self.wheel_control_publisher = self.create_publisher(String, 'wheel_controller_command', 10)
        
        self.log_subscriber = self.create_subscription(
            String, '/log_publisher', self.log_callback, 10
        )

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.spin_ros)
        self.timer.start(100)

        # Connect the log signal to the log display method
        self.log_signal.connect(self.display_log_message)
        
        # Connect
        # Activation
        self.ui.activation_robot2_button.clicked.connect(lambda: self.activate_robot2(self.ui.activation_robot2_button))
        self.ui.activation_joint_button.clicked.connect(lambda: self.activate_joint(self.ui.activation_joint_button))
        self.ui.activation_all_button.clicked.connect(lambda: self.activate_all(self.ui.activation_all_button))
        
        # Calibration
        self.ui.calibration_all_button.clicked.connect(self.calibrate_all)
        self.ui.calibration_robot2_button.clicked.connect(self.calibrate_robot2)
        self.ui.calibration_joint_button.clicked.connect(self.calibrate_joint)
        self.ui.calibration_wheel_button.clicked.connect(self.calibrate_wheel)
        
        # robot2 control
        self.ui.robot2_control_both_up_arrow_button.clicked.connect(lambda: self.robot2_command(self.ui.robot2_control_both_up_arrow_button, "both_forward"))
        self.ui.robot2_control_both_down_arrow_button.clicked.connect(lambda: self.robot2_command(self.ui.robot2_control_both_down_arrow_button, "both_backward"))
        self.ui.robot2_control_both_left_arrow_button.clicked.connect(lambda: self.robot2_command(self.ui.robot2_control_both_left_arrow_button, "both_left"))
        self.ui.robot2_control_both_right_arrow_button.clicked.connect(lambda: self.robot2_command(self.ui.robot2_control_both_right_arrow_button, "both_right"))
        self.ui.robot2_control_left_up_arrow_button.clicked.connect(lambda: self.robot2_command(self.ui.robot2_control_left_up_arrow_button, "left_forward"))
        self.ui.robot2_control_left_down_arrow_button.clicked.connect(lambda: self.robot2_command(self.ui.robot2_control_left_down_arrow_button, "left_backward"))
        self.ui.robot2_control_right_up_arrow_button.clicked.connect(lambda: self.robot2_command(self.ui.robot2_control_right_up_arrow_button, "right_forward"))
        self.ui.robot2_control_right_down_arrow_button.clicked.connect(lambda: self.robot2_command(self.ui.robot2_control_right_down_arrow_button, "right_backward"))
        
        
        # robot1 remote
        self.ui.button_up_robot1.clicked.connect(lambda: self.robot1_remote(self.ui.button_up_robot1, "upward"))
        self.ui.button_down_robot1.clicked.connect(lambda: self.robot1_remote(self.ui.button_down_robot1, "downward"))
        self.ui.button_right_robot1.clicked.connect(lambda: self.robot1_remote(self.ui.button_right_robot1, "right"))
        self.ui.button_left_robot1.clicked.connect(lambda: self.robot1_remote(self.ui.button_left_robot1, "left"))
        self.ui.button_up_right_robot1.clicked.connect(lambda: self.robot1_remote(self.ui.button_up_right_robot1, "up_right"))
        self.ui.button_up_left_robot1.clicked.connect(lambda: self.robot1_remote(self.ui.button_up_left_robot1, "up_left"))
        self.ui.button_down_right_robot1.clicked.connect(lambda: self.robot1_remote(self.ui.button_down_right_robot1, "down_right"))
        self.ui.button_down_left_robot1.clicked.connect(lambda: self.robot1_remote(self.ui.button_down_left_robot1, "down_left"))
        
        # Mode
        self.ui.stand_button.clicked.connect(self.stand_mode)
        self.ui.default_button.clicked.connect(self.default_mode)
        self.ui.land_button.clicked.connect(self.land_mode)
        self.ui.posture_A_button.clicked.connect(self.posture_A_mode)
        self.ui.posture_B_button.clicked.connect(self.posture_B_mode)

        # Pose control
        self.ui.set_button.clicked.connect(self.set_adjust_command)
        
        # Segment stop
        self.ui.test_1_button.clicked.connect(self.stop_segment_command)

        # Wheel control
        self.ui.wheel_control_button.clicked.connect(self.activate_all_wheels)
        self.ui.wheel_control_FL_icon_button.clicked.connect(lambda: self.toggle_individual_wheel(self.ui.wheel_control_FL_icon_button))
        self.ui.wheel_control_FR_icon_button.clicked.connect(lambda: self.toggle_individual_wheel(self.ui.wheel_control_FR_icon_button))
        self.ui.wheel_control_RL_icon_button.clicked.connect(lambda: self.toggle_individual_wheel(self.ui.wheel_control_RL_icon_button))
        self.ui.wheel_control_RR_icon_button.clicked.connect(lambda: self.toggle_individual_wheel(self.ui.wheel_control_RR_icon_button))

        
        self.ui.wheel_velocity_set_button.clicked.connect(self.set_wheel_velocity)
        
        
        self.ui.wheel_control_front_button.clicked.connect(lambda: self.wheel_command(self.ui.wheel_control_front_button, "forward"))
        self.ui.wheel_control_back_button.clicked.connect(lambda: self.wheel_command(self.ui.wheel_control_back_button, "backward"))
        self.ui.wheel_control_stop_button.clicked.connect(lambda: self.wheel_command(self.ui.wheel_control_stop_button, "stop"))

        self.ui.wheel_control_FL_plus_button.clicked.connect(lambda: self.wheel_command(self.ui.wheel_control_FL_plus_button, "fl_forward"))
        self.ui.wheel_control_FL_minus_button.clicked.connect(lambda: self.wheel_command(self.ui.wheel_control_FL_minus_button, "fl_backward"))

        self.ui.wheel_control_FR_plus_button.clicked.connect(lambda: self.wheel_command(self.ui.wheel_control_FR_plus_button, "fr_forward"))
        self.ui.wheel_control_FR_minus_button.clicked.connect(lambda: self.wheel_command(self.ui.wheel_control_FR_minus_button, "fr_backward"))

        self.ui.wheel_control_RL_plus_button.clicked.connect(lambda: self.wheel_command(self.ui.wheel_control_RL_plus_button, "rl_forward"))
        self.ui.wheel_control_RL_minus_button.clicked.connect(lambda: self.wheel_command(self.ui.wheel_control_RL_minus_button, "rl_backward"))

        self.ui.wheel_control_RR_plus_button.clicked.connect(lambda: self.wheel_command(self.ui.wheel_control_RR_plus_button, "rr_forward"))
        self.ui.wheel_control_RR_minus_button.clicked.connect(lambda: self.wheel_command(self.ui.wheel_control_RR_minus_button, "rr_backward"))
    # Activation



    def activate_all_wheels(self):
        global_state = self.ui.wheel_control_button.property("state")
        
        new_state = "true" if global_state == "false" else "false"
        
        self.ui.wheel_control_button.setProperty("state", new_state)
        self.ui.toggleButtonStyle(self.ui.wheel_control_button)

        for button in [self.ui.wheel_control_FL_icon_button, self.ui.wheel_control_FR_icon_button, 
                       self.ui.wheel_control_RL_icon_button, self.ui.wheel_control_RR_icon_button]:
            button.setProperty("state", new_state)
            self.ui.toggleButtonStyle(button)

        self.publish_wheel_states()

    def toggle_individual_wheel(self, button):
        current_state = button.property("state")
        new_state = "true" if current_state == "false" else "false"
        button.setProperty("state", new_state)
        self.ui.toggleButtonStyle(button)

        self.publish_wheel_states()

    def publish_wheel_states(self):
        wheel_states = [
            self.ui.wheel_control_FL_icon_button.property("state") == "true",
            self.ui.wheel_control_FR_icon_button.property("state") == "true",
            self.ui.wheel_control_RL_icon_button.property("state") == "true",
            self.ui.wheel_control_RR_icon_button.property("state") == "true"
        ]

        msg = BridgeBoolMultiArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['is_wheel_activation']
        msg.bool_values = wheel_states
        self.wheel_activation_pub.publish(msg)

        # Log the updated states
        states_description = ", ".join(f"Wheel {i+1}: {'On' if state else 'Off'}" for i, state in enumerate(wheel_states))
        self.ui.system_log_browser.append(f"Updated wheel states: {states_description}")


    def activate_robot2(self, button):
        current_state = button.property("state")
        # Toggle the current state and update the button appearance

        if current_state == "true":
            activation_state = True
        else:
            activation_state = False
        
        msg = BridgeBoolMultiArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['is_robot2_activation']
        msg.bool_values = [activation_state]  # Make sure it's a list
        self.robot2_activation_pub.publish(msg)
        
        # Log the action to the system log browser
        self.ui.system_log_browser.append(f"robot2 activation set to {'Activated' if activation_state else 'Deactivated'}")

    def activate_joint(self, button):
        current_state = button.property("state")
        # Toggle the current state and update the button appearance

        if current_state == "true":
            activation_state = True
        else:
            activation_state = False
        
        msg = BridgeBoolMultiArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['is_joint_activation']
        msg.bool_values = [activation_state]  # Make sure it's a list
        self.joint_activation_pub.publish(msg)
        
        # Log the action to the system log browser
        self.ui.system_log_browser.append(f"Joint activation set to {'Activated' if activation_state else 'Deactivated'}")

    def activate_all(self, button):
        current_state = button.property("state")
        # Toggle the current state and update the button appearance

        if current_state == "true":
            activation_state = True
        else:
            activation_state = False
        
        msg = BridgeBoolMultiArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['is_robot_activation']
        msg.bool_values = [activation_state]  # Make sure it's a list
        self.robot_activation_pub.publish(msg)
        
        # Log the action to the system log browser
        self.ui.system_log_browser.append(f"Robot activation set to {'Activated' if activation_state else 'Deactivated'}")



    # Calibrate
    
    def stop_segment_command(self):
        print("Button clicked!")  # Debugging line
        msg = String()
        msg.data = "segment_stop"
        self.segment_stop_publisher.publish(msg)
        self.ui.system_log_browser.append("Published: segment_stop")

    
    def calibrate_robot2(self):
        msg = String()
        msg.data = "robot2 Calibrate"
        self.calibration_publisher.publish(msg)
        self.ui.system_log_browser.append("Published: robot2 Calibrated")

    def calibrate_joint(self):
        msg = String()
        msg.data = "Joint Calibrate"
        self.calibration_publisher.publish(msg)
        self.ui.system_log_browser.append("Published: Joint Calibrated")

    def calibrate_wheel(self):
        msg = String()
        msg.data = "Wheel Calibrate"
        self.calibration_publisher.publish(msg)
        self.ui.system_log_browser.append("Published: Wheel Calibrated")

    def calibrate_all(self):
        msg = String()
        msg.data = "All Calibrate"
        self.calibration_publisher.publish(msg)
        self.ui.system_log_browser.append("Published: All Calibrated")

    # System Log

    def log_callback(self, msg):
        """Callback for ROS2 log messages."""
        self.log_signal.emit(msg.data)

    def display_log_message(self, message):
        """Display log messages in the text browser."""
        self.ui.system_log_browser.append(message)
        self.ui.system_log_browser.verticalScrollBar().setValue(self.ui.system_log_browser.verticalScrollBar().maximum())

    def spin_ros(self):
        rclpy.spin_once(self, timeout_sec=0.1)


    # robot2 Control

    def robot2_command(self, button, command):
        current_state = button.property("state")
        msg = String()

        if current_state == "true":
            msg.data = command
        else:
            msg.data = "stop"

        self.robot2_command_publisher.publish(msg)
        self.ui.system_log_browser.append(f"robot2 command : {msg.data}")

    def robot1_remote(self, button, command):
        current_state = button.property("state")
        msg = String()
        
        if current_state == "true":
            msg.data = command
        else:
            msg.data = "stop"
            
        self.stand_command_publisher.publish(msg)
        self.ui.system_log_browser.append(f"robot1 remote command : {msg.data}")
            
        
    # Mode

    def stand_mode(self):
        msg = String()
        msg.data = "stand"
        self.robot1_command_publisher.publish(msg)
        #self.ui.system_log_browser.append("stand")

    def default_mode(self):
        msg = String()
        msg.data = "default"
        self.robot1_command_publisher.publish(msg)
        #self.ui.system_log_browser.append("default")

    def land_mode(self):
        msg = String()
        msg.data = "land"
        self.robot1_command_publisher.publish(msg)
        #self.ui.system_log_browser.append("land")

    def posture_A_mode(self):
        msg = String()
        msg.data = "sequence_first"
        self.robot1_command_publisher.publish(msg)
        #self.ui.system_log_browser.append("posture1")
        
    def posture_B_mode(self):
        msg = String()
        msg.data = "sequence_second"
        self.robot1_command_publisher.publish(msg)
        #self.ui.system_log_browser.append("posture2")
        

    def set_adjust_command(self):
        try:
            dx = float(self.ui.pose_control_dx_textedit.toPlainText().strip())
            dy = float(self.ui.pose_control_dy_textedit.toPlainText().strip())
            dz = float(self.ui.pose_control_dz_textedit.toPlainText().strip())
            msg = String()
            msg.data = f"adjust {dx} {dy} {dz}"
            self.robot1_command_publisher.publish(msg)
            self.ui.system_log_browser.append(f"Published: {msg.data}")
        except ValueError:
            self.ui.system_log_browser.append("Error: Invalid or incomplete input for dx, dy, dz.")

    # Wheel control


    def wheel_command(self, button, command):
        current_state = button.property("state")
        msg = String()
        

        msg.data = command


        self.wheel_control_publisher.publish(msg)  # Assumes a publisher for wheel commands
        self.ui.system_log_browser.append(f"Wheel command sent: {msg.data}")

    def set_wheel_velocity(self):
        velocity = self.ui.wheel_control_velocity_textedit.toPlainText()
        try:
            # Update the velocity in the wheel_command object
            self.wheel_velocity_command = velocity
            self.ui.system_log_browser.append(f"Velocity set to: {velocity}")

        except ValueError as e:
            self.ui.system_log_browser.append(f"Error setting velocity: {str(e)}")
    #def wheel_forward

def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    ui_controller = UIController()
    ui_controller.show()
    exit_code = app.exec_()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
