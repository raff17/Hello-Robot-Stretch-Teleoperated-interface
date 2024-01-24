#!/usr/bin/env python3
import typing
import rospy
import sys
import os
import cv2
import logging
import time
import shapely
import actionlib
import threading
from sensor_msgs.msg import Image, JointState
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


#QT command 
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QMainWindow, QStackedWidget
from PyQt5.QtCore import QTimer
from PyQt5.QtCore import Qt
# rospy.init_node("click")

class NavigationController():

    '''This is used to control the base motion of stretch'''
    def __init__(self, forwards_scale = 1, backwards_scale = 1, left_scale = 1, right_scale = 1, parent=None):
        self.areas = [
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (1, 0)]),
                "callback" : self.go_forwards,
                "mouseover": self.mouseover_forwards
            },
            {
                "geometry" : shapely.Polygon([(0, 1), (0.5, 0.5), (1, 1)]),
                "callback" : self.go_backwards,
                "mouseover": self.mouseover_backwards
            },
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (0, 1)]),
                "callback" : self.turn_left,
                "mouseover": self.mouseover_left
            },
            {
                "geometry" : shapely.Polygon([(1, 0), (0.5, 0.5), (1, 1)]),
                "callback" : self.turn_right,
                "mouseover": self.mouseover_right
            }
        ]
        self.parent = parent
        self.move_publisher = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=5)
    def mouseover_event(self, event):
        return
        """dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["mouseover"]()"""

    def click_event(self, event):
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
        

    def go_forwards(self):
        msg = Twist()
        msg.linear.x = 2
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.linear.x = 0
        self.move_publisher.publish(msg)
        logging.info("forward")

    def go_backwards(self):
        msg = Twist()
        msg.linear.x = -2
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.linear.x = 0
        self.move_publisher.publish(msg)
        logging.info("back")
    
    def turn_left(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 2
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.angular.z = 0
        self.move_publisher.publish(msg)

    def turn_right(self):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = -2
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.angular.z = 0
        self.move_publisher.publish(msg)

    def mouseover_forwards(self):
        print("Mouseover forwards")
    
    def mouseover_backwards(self):
        print("Mouseover backwards")
    
    def mouseover_left(self):
        print("Mouseover left")
    
    def mouseover_right(self):
        print("Mouseover right")


class ArmController():
    '''Controls for the arm! (NOT GRIPPER)'''
    def __init__(self, forwards_scale = 1, backwards_scale = 1, left_scale = 1, right_scale = 1, parent=None):
        self.areas = [
            # This actually extends
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (1, 0)]),
                "callback" : self.retract
            },
            # This retracts
            {
                "geometry" : shapely.Polygon([(0, 1), (0.5, 0.5), (1, 1)]),
                "callback" : self.extend
            },
            # This moves the base arm
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (0, 1)]),
                "callback" : self.move_up
            },
            # This moves the arm down
            {
                "geometry" : shapely.Polygon([(1, 0), (0.5, 0.5), (1, 1)]),
                "callback" : self.move_down
            }
        ]
        self.delt_vert = 0.02
        self.delt_horiz = 0.02
        self.parent = parent
        self.joint_states = None

        #For real
        #self.arm_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        #For simulation
        self.arm_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)

        server_reached = self.arm_client.wait_for_server(timeout=rospy.Duration(60.0))
        self.joints_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        #self.head_publisher = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=5)

    def joint_states_cb(self, data):
        self.joint_states = data

    def mouseover_event(self, event):
        pass

    def click_event(self, event):
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
                QTimer.singleShot(2000)
        

    def move_up(self):
        command = {'joint': 'joint_lift', 'delta': self.delt_vert}
        self.send_command(command)

    def move_down(self):
        command = {'joint': 'joint_lift', 'delta': -self.delt_vert}
        self.send_command(command)

    def extend(self):
        command = {'joint': ['joint_arm_l0','joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'], 'delta': -self.delt_horiz}
        self.send_arm_command(command)

    def retract(self):
        command = {'joint': ['joint_arm_l0','joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'], 'delta': self.delt_horiz}
        self.send_arm_command(command)

    def send_command(self, command):
        while self.joint_states == None:
            time.sleep(0.1)
        try:
            joint_state = self.joint_states
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.1)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(0.25)
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            joint_index = joint_state.name.index(joint_name)
            joint_value = joint_state.position[joint_index]
            delta = command['delta']
            new_value = joint_value + delta
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.arm_client.send_goal(trajectory_goal)
        except Exception as ex:
            logging.error("Error: Exception encountered while passing command {command} to camera controls")
            logging.error(ex)
        
    def send_arm_command(self, command):
        while self.joint_states == None:
            time.sleep(0.1)
        try:
            joint_state = self.joint_states
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.1)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(0.25)
            trajectory_goal.trajectory.joint_names = command['joint']
            point.positions = []
            for j_name in trajectory_goal.trajectory.joint_names:
                joint_index = joint_state.name.index(j_name)
                joint_value = joint_state.position[joint_index]
                delta = command['delta']
                new_value = joint_value + delta/len(trajectory_goal.trajectory.joint_names)
                point.positions.append(new_value)
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.arm_client.send_goal(trajectory_goal)
        except Exception as ex:
            logging.error("Error: Exception encountered while passing command {command} to camera controls")
            logging.error(ex)

    def mouseover_forwards(self):
        print("Mouseover forwards")
    
    def mouseover_backwards(self):
        print("Mouseover backwards")
    
    def mouseover_left(self):
        print("Mouseover left")
    
    def mouseover_right(self):
        print("Mouseover right")


class GripperController():
    '''Controls for the gripper'''
    def __init__(self, forwards_scale = 1, backwards_scale = 1, left_scale = 1, right_scale = 1, parent=None, dex_wrist=False):
        if not dex_wrist:
            self.areas = [
                {
                    "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (1, 0)]),
                    "callback" : self.open_gripper
                },
                {
                    "geometry" : shapely.Polygon([(0, 1), (0.5, 0.5), (1, 1)]),
                    "callback" : self.close_gripper
                },
                {
                    "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (0, 1)]),
                    "callback" : self.turn_left
                },
                {
                    "geometry" : shapely.Polygon([(1, 0), (0.5, 0.5), (1, 1)]),
                    "callback" : self.turn_right
                }
            ]
        self.grip_factor = 0.05
        self.yaw_factor = 0.1
        self.parent = parent
        self.joint_states = None
        self.arm_client = actionlib.SimpleActionClient('/stretch_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        server_reached = self.arm_client.wait_for_server(timeout=rospy.Duration(60.0))
        self.joints_subscriber = rospy.Subscriber('/joint_states', JointState, self.joint_states_cb)
        #self.head_publisher = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=5)
    def joint_states_cb(self, data):
        self.joint_states = data

    def mouseover_event(self, event):
        pass

    def click_event(self, event):
        self.autorepeat()
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()
        point = shapely.Point((norm_x, norm_y))
        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
        

    def open_gripper(self):
        command = {'joint': 'joint_gripper_finger_left', 'delta': self.grip_factor}
        self.send_command(command)
        print("open gripper")
        logging.info(f"Open gripper")
    def close_gripper(self):
        command = {'joint': 'joint_gripper_finger_left', 'delta': -self.grip_factor}
        self.send_command(command)
        logging.info("Close gripper")
        print("close gripper")

    def turn_left(self):
        command = {'joint': 'joint_wrist_yaw', 'delta': self.yaw_factor}
        self.send_command(command)
        logging.info("left gripper")

    def turn_right(self):
        command = {'joint': 'joint_wrist_yaw', 'delta': -self.yaw_factor}
        self.send_command(command)
        logging.info("right gripper")

    def send_command(self, command):
        while self.joint_states == None:
            time.sleep(0.1)
        try:
            joint_state = self.joint_states
            point = JointTrajectoryPoint()
            point.time_from_start = rospy.Duration(0.1)
            trajectory_goal = FollowJointTrajectoryGoal()
            trajectory_goal.goal_time_tolerance = rospy.Time(0.25)
            joint_name = command['joint']
            trajectory_goal.trajectory.joint_names = [joint_name]
            joint_index = joint_state.name.index(joint_name)
            joint_value = joint_state.position[joint_index]
            delta = command['delta']
            new_value = joint_value + delta
            point.positions = [new_value]
            trajectory_goal.trajectory.points = [point]
            trajectory_goal.trajectory.header.stamp = rospy.Time.now()
            self.arm_client.send_goal(trajectory_goal)
        except Exception as ex:
            logging.error("Error: Exception encountered while passing command {command} to camera controls")
            logging.error(ex)
        

    def mouseover_forwards(self):
        print("Mouseover forwards")
    
    def mouseover_backwards(self):
        print("Mouseover backwards")
    
    def mouseover_left(self):
        print("Mouseover left")
    
    def mouseover_right(self):
        print("Mouseover right")

#######################################################################


class SquareButton(QPushButton):
    ''' Creates buttons (this section you select the size and shape)'''
    def __init__(self, parent=None, dimensions=(430, 173)):
        super(QPushButton, self).__init__(parent)
        self.setCheckable(True)
        self.autoRepeat()
        self.setFixedSize(dimensions[0], dimensions[1])

class Button(QPushButton):
    ''' Creates buttons (this section you select the size and shape)'''
    def __init__(self, parent=None, dimensions=(730, 94)):
        super(QPushButton, self).__init__(parent)
        self.setFixedSize(dimensions[0], dimensions[1])


class GripperButtonset(QWidget):
    '''Assigns the buttons you made to specific actions (gripper movement)'''
    def __init__(self, parent=None):
        super(GripperButtonset, self).__init__(parent)
        self.controller = GripperController(parent=self)
        # self.up_button = SquareButton(parent=self)
        # self.up_button.setText("Open Gripper")
        # self.down_button = SquareButton(parent=self)
        # self.down_button.setText("Close Gripper")
        self.left_button = SquareButton(parent=self)
        self.pointSize = 60                                     
        self.fontD = self.font()
        self.fontD.setPointSize(self.pointSize)
        self.left_button.setFont(self.fontD) 
        self.left_button.setText("Hand Left")
        self.right_button = SquareButton(parent=self)
        self.right_button.setFont(self.fontD)
        self.right_button.setText("Hand Right")

        # self.up_button.clicked.connect(self.controller.open_gripper)
        # self.down_button.clicked.connect(self.controller.close_gripper)
        self.left_button.clicked.connect(self.controller.turn_left)
        self.right_button.clicked.connect(self.controller.turn_right)
        self.right_button.setCheckable(True)

        #This part takes the button creation and made them to QT interface
        self.main_layout = QVBoxLayout()
        self.middle_widget = QWidget()
        self.middle_layout = QHBoxLayout()

        self.middle_layout.addWidget(self.left_button)
        self.middle_layout.addWidget(self.right_button)
        self.middle_layout.setAlignment(self.left_button, Qt.AlignLeft)
        self.middle_layout.setAlignment(self.right_button, Qt.AlignRight)
        self.middle_widget.setLayout(self.middle_layout)

        # self.main_layout.addWidget(self.up_button)
        self.main_layout.addWidget(self.middle_widget)
        # self.main_layout.addWidget(self.down_button)

        # self.main_layout.setAlignment(self.up_button, Qt.AlignTop)
        # self.main_layout.setAlignment(self.down_button, Qt.AlignBottom)

        # self.main_layout.setAlignment(self.up_button, Qt.AlignCenter)
        # self.main_layout.setAlignment(self.down_button, Qt.AlignCenter)

        self.setLayout(self.main_layout)
        self.setFixedSize(900, 200)
        # self.setFixedSize(1200,130)

class Buttonset(QWidget):
    '''Assigns the buttons you made to specific actions (gripper movement)'''
    def __init__(self, parent=None):
        super(Buttonset, self).__init__(parent)
        self.controller = GripperController(parent=self)
        self.pointSize = 60                                     
        self.fontD = self.font()
        self.fontD.setPointSize(self.pointSize)

        self.up_button = Button(parent=self)
        self.up_button.setFont(self.fontD)
        self.up_button.setText("Open Gripper")
        self.down_button = Button(parent=self)
        self.down_button.setFont(self.fontD)
        self.down_button.setText("Close Gripper")
        # self.left_button = SquareButton(parent=self)
        # self.left_button.setText("Turn Left")
        # self.right_button = SquareButton(parent=self)
        # self.right_button.setText("Turn Right")

        self.up_button.clicked.connect(self.controller.open_gripper)
        logging.info('Open gripper')
        self.down_button.clicked.connect(self.controller.close_gripper)
        # self.left_button.clicked.connect(self.controller.turn_left)
        # self.right_button.clicked.connect(self.controller.turn_right)

        #This part takes the button creation and made them to QT interface
        self.main_layout = QVBoxLayout()
        self.middle_widget = QWidget()
        self.middle_layout = QHBoxLayout()

        # self.middle_layout.addWidget(self.left_button)
        # self.middle_layout.addWidget(self.right_button)
        # self.middle_layout.setAlignment(self.left_button, Qt.AlignLeft)
        # self.middle_layout.setAlignment(self.right_button, Qt.AlignRight)
        # self.middle_widget.setLayout(self.middle_layout)

        self.main_layout.addWidget(self.up_button)
        self.main_layout.addWidget(self.middle_widget)
        self.main_layout.addWidget(self.down_button)

        self.main_layout.setAlignment(self.up_button, Qt.AlignTop)
        self.main_layout.setAlignment(self.down_button, Qt.AlignBottom)

        self.main_layout.setAlignment(self.up_button, Qt.AlignCenter)
        self.main_layout.setAlignment(self.down_button, Qt.AlignCenter)

        self.setLayout(self.main_layout)
        self.setFixedSize(750, 200)
        if self.up_button:
            logging.info("Open gripper")


class DisplayImageWidget(QWidget):
    def __init__(self, parent=None):
        super(DisplayImageWidget, self).__init__(parent)

        #self.button = QPushButton('Show picture')
        #self.button.clicked.connect(self.show_image)
        self.image_frame = QLabel()
        self.image_frame.setFixedSize(432, 768)

        #Needed to handle mouse move events
        self.setMouseTracking(True)

        self.nav_controller = NavigationController(parent=self)
        self.arm_controller = ArmController(parent=self)
        self.grip_controller = GripperController(parent=self)

        self.mode = "camera"

        self.available_modes = {
            "camera" : {
                "show_function" : self.only_show_image,
                "controller" : None
            }
        }

        self.image_frame.mousePressEvent = self.click_event
        self.setAttribute(QtCore.Qt.WA_Hover)
        #self.image_frame.mouseMoveEvent = self.mouseover_event


        self.layout = QVBoxLayout()
        self.layout.addWidget(self.image_frame)
        self.setLayout(self.layout)

    def set_mode(self, mode):
        if mode in self.available_modes.keys():
            self.mode = mode
            logging.debug(f"Switching to mode {mode}")
        else:
            self.mode = "camera"
            logging.warning(f"Interface has no mode \"{mode}\"")

    def mouseMoveEvent(self, event):
        if self.available_modes[self.mode]["controller"] != None:
            self.available_modes[self.mode]["controller"].mouseover_event(event)

    def click_event(self, event):
        if self.available_modes[self.mode]["controller"] != None:
            self.available_modes[self.mode]["controller"].click_event(event)
#############################################################################################################
# This shows the camera image
    def only_show_image(self, cv_image):
        cv_image = cv2.resize(cv_image, (self.image_frame.width(), self.image_frame.height()), interpolation=cv2.INTER_LINEAR)
        cv_image = self.draw_shapes(cv_image)
        self.show_image(QtGui.QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], QtGui.QImage.Format_RGB888))

    def show_navigation(self, cv_image):
        cv_image = cv2.resize(cv_image, (self.image_frame.width(), self.image_frame.height()), interpolation=cv2.INTER_LINEAR)
        cv_image = self.draw_shapes(cv_image)
        self.show_image(QtGui.QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], QtGui.QImage.Format_RGB888))


    def only_show_image2(self, cv_image):
        cv_image = cv2.resize(cv_image, (self.image_frame.width(), self.image_frame.height()), interpolation=cv2.INTER_LINEAR)
        cv_image = self.draw_shapes2(cv_image)
        self.show_image(QtGui.QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], QtGui.QImage.Format_RGB888))

    def show_navigation2(self, cv_image):
        cv_image = cv2.resize(cv_image, (self.image_frame.width(), self.image_frame.height()), interpolation=cv2.INTER_LINEAR)
        cv_image = self.draw_shapes2(cv_image)
        self.show_image(QtGui.QImage(cv_image.data, cv_image.shape[1], cv_image.shape[0], QtGui.QImage.Format_RGB888))



################################################################################################################

    def draw_shapes(self, cv_image):
        overlay = cv_image.copy()
        alpha = 0.5
        dimensions = self.image_frame.size()
        w = dimensions.width()
        h = dimensions.height()
        for area in self.available_modes[self.mode]["controller"].areas:
            points = area["geometry"].exterior.coords
            for i in range(len(points)-1):
                p1 = points[i+1]
                p2 = points[i]
                cv2.line(overlay, (int(p1[0]*w), int(p1[1]*h)), (int(p2[0]*w), int(p2[1]*h)), (255, 255, 255), 6)
        cv_image = cv2.addWeighted(overlay, alpha, cv_image, 1-alpha, 0)
        return cv_image
    
    def show_image_by_mode(self, cv_image):
        self.available_modes[self.mode]["show_function"](cv_image)

    @QtCore.pyqtSlot()
    def show_image(self, image):
        self.image_frame.setPixmap(QtGui.QPixmap.fromImage(image))



class ManipulationPage(QWidget):
    '''Creates a page call Manipulation and display specific information there'''
    def __init__(self):
        super(QWidget, self).__init__()
        # self.vid_bridge = CvBridge()
        self.label = QLabel('Arm In', self)
        self.label.setAlignment(Qt.AlignHCenter)
        self.label.setFixedHeight(900)
        self.label.setAlignment(Qt.AlignRight)
        self.label.setFixedWidth(900)
        self.layout = QVBoxLayout()
        #self.top_label = QLabel(text="Manipulator Mode")
        #self.top_label.setMaximumHeight(20)
        # self.GRIPPER_widget = DisplayImageWidget(parent=self)
        #####################################################################
        #basemotion/attach camera view
        self.arm_widget = QWidget()
        self.arm_layout = QHBoxLayout()

        self.arm_camera_widget = QWidget()
        self.arm_camera_label = QLabel(text="Base movement")
        # self.arm_camera_label.setFixedHeight(20)
        self.arm_camera_label.setFixedSize(20, 40)


        # changing this is will change what the clicking on the camera actually does
        #This should be arm movement
        # This controls the size of the image shown and what is connected to the clicking for that image
        self.arm_camera = DisplayImageWidget(parent=self)
        self.arm_camera.image_frame.setFixedSize(800, 700)
        #This area is what connect the camera to the position "gripper" will open up the camera that is name in gripper
        self.arm_camera.available_modes["arm"] = {"show_function" : self.arm_camera.show_navigation, "controller" : NavigationController(parent=self.arm_camera)}
        self.arm_camera.set_mode("arm")


        self.arm_camera_layout = QVBoxLayout()
        self.arm_camera_layout.addWidget(self.arm_camera_label)
        self.arm_camera_layout.addWidget(self.arm_camera)

        self.cam_buttons = GripperButtonset(parent=self)

        self.arm_camera_layout.addWidget(self.cam_buttons)
        self.arm_camera_layout.setAlignment(self.arm_camera_label, Qt.AlignHCenter)
        self.arm_camera_widget.setLayout(self.arm_camera_layout)
        self.arm_camera_layout.setAlignment(self.cam_buttons, Qt.AlignCenter)

        self.arm_cam_thread = threading.Thread(target=self.run_arm_camera, daemon=True)
        self.arm_cam_thread.start()

        #####################################################################
        #Armmotion/gripper camera view
        self.gripper_widget = QWidget()
        self.gripper_layout = QHBoxLayout()

        self.gripper_camera_widget = QWidget()
        self.gripper_camera_label = QLabel(text="Arm Movement")
        self.gripper_camera_label.setFixedHeight(20)

        self.gripper_camera = DisplayImageWidget(parent=self)
        self.gripper_camera.image_frame.setFixedSize(800, 700)
        #This area is what connect the camera to the position "gripper" will open up the camera that is name in gripper
        self.gripper_camera.available_modes["gripper"] = {"show_function" : self.gripper_camera.show_navigation, "controller" : ArmController(parent=self.gripper_camera)}
        self.gripper_camera.set_mode("gripper")
        self.gripper_camera_layout = QVBoxLayout()
        self.gripper_camera_layout.addWidget(self.gripper_camera_label)
        self.gripper_camera_layout.addWidget(self.gripper_camera)

        self.cam_buttonss = Buttonset(parent=self)
        if self.cam_buttonss:
                print(self.cam_buttonss)
                logging.info(Buttonset(parent=self))
        
        self.gripper_camera_layout.addStretch()
        self.gripper_camera_layout.addWidget(self.cam_buttonss)
        self.gripper_camera_layout.setAlignment(self.gripper_camera_label, Qt.AlignHCenter)



        self.gripper_camera_widget.setLayout(self.gripper_camera_layout)
        self.gripper_camera_layout.setAlignment(self.cam_buttonss, Qt.AlignCenter)

        self.gripper_cam_thread = threading.Thread(target=self.main_camera_cb, daemon=True)
        self.gripper_cam_thread.start()

# possibly change ----------------------------------------------------------------

        self.arm_layout.addWidget(self.arm_camera_widget)
        self.arm_layout.addWidget(self.gripper_camera_widget)
        # self.arm_layout.addWidget(self.cam_buttons)
        # self.arm_layout.setAlignment(self.cam_buttons, Qt.AlignCenter)
        self.arm_widget.setLayout(self.arm_layout)

        self.layout.addWidget(self.arm_widget)
        # self.gripper_layout.addWidget(self.gripper_camera_widget)
        # self.arm_widget.setLayout(self.arm_layout)
        # self.gripper_layout.addWidget(self.gripper_camera_widget)
        # self.gripper_widget.setLayout(self.gripper_layout)
        # self.arm_widget.setLayout(self.arm_layout)

        # self.navigation_button = QPushButton(text="Navigation Mode")
        # self.navigation_button.setFixedHeight(80)

        #self.layout.addWidget(self.top_label)
        # self.layout.addWidget(self.gripper_widget)
        # self.layout.addWidget(self.arm_widget)
        # self.layout.addWidget(self.gripper_widget)
        # self.layout.addWidget(self.navigation_button)
        self.layout.addStretch()

        # self.layout.addChildWidget(self.arm_widget)

        self.setLayout(self.layout)


        #######################################################################         
    # Adding things selects the camera  
    # camera for gripper       
    def main_camera_cb(self):
        vid = cv2.VideoCapture(8)
        #vid = cv2.VideoCapture(0)
        while(True):
            # Capture the video frame
            # by frame
            _, frame = vid.read()
            self.gripper_camera.show_image_by_mode(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
#####################################################################
# Camera for movement (webcam)
    def run_arm_camera(self):
        vid = cv2.VideoCapture(6)
        #vid = cv2.VideoCapture(0)
        while(True):
            # Capture the video frame
            # by frame
            _, frame = vid.read()
            self.arm_camera.show_image_by_mode(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

if __name__=="__main__":
    logging.basicConfig(format='%(levelname)s %(asctime)s: %(message)s',filename=f"{os.path.dirname(os.path.abspath(__file__))}/logs/interface_data_{time.strftime('%y_%m_%d:%H_%M_%S', time.localtime(time.time()))}.log", level=logging.INFO, datefmt="%y-%m-%d:%h-%m-%s")
    # logging.info("Starting application")
    logging.info("Starting application")
    rospy.init_node("click")

    # rospy.init_node("interface_data", anonymous=True)

    app = QApplication(sys.argv)

    window = ManipulationPage()
    window.show()

    app.exec()
