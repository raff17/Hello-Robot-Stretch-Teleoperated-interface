#!/usr/bin/env python3
"""
Author: Rafael Morales Mayoral
Institution: Oregon State University 
About code: This code is used to teleoperate the Hello-Robot RE2. It uses PYQT5 to create an interface that can be control via computer OR can also be VNC to a advice (such as a tablet)
"""

#require imports
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
from threading import Thread
import pygame

#Sounds
from playsound import playsound


#QT command 
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QPushButton, QVBoxLayout, QHBoxLayout, QMainWindow, QStackedWidget
from PyQt5.QtCore import Qt
sys.stdout = open(f"{os.path.dirname(os.path.abspath(__file__))}/logs/input_{time.strftime('%y_%m_%d:%H_%M_%S', time.localtime(time.time()))}.log",'wt')


class NavigationController():
    '''This is used to control the base motion of stretch by detecting the desire movement and moving towards that direction. The polygon shapes are used for the seperation of action sections'''
    def __init__(self, forwards_scale = 1, backwards_scale = 1, left_scale = 1, right_scale = 1, parent=None):
        self.areas = [
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (1, 0)]),
                "callback" : self.go_forwards
            },
            {
                "geometry" : shapely.Polygon([(0, 1), (0.5, 0.5), (1, 1)]),
                "callback" : self.go_backwards
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

        # Timer for the movement sounds (in order for)
        self.timer_forward = 0
        self.timer_back = 0
        self.timer_left_turn = 0
        self.timer_right_turn = 0

        self.parent = parent
        self.move_publisher = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=5)

        # TODO: get the mouse function working in order to allow for a single press movement instead of constant clicks.
    def mouseover_event(self, event):
        return


    def click_event(self, event):
        '''Detects when the mouse clicks on the screen'''
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()
        point = shapely.Point((norm_x, norm_y))
        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
        

    # def click_event(self, event):
    #     dimensions = self.parent.image_frame.frameGeometry()
    #     norm_x = event.x() / dimensions.width()
    #     norm_y = event.y() / dimensions.height()
    #     point = shapely.Point((norm_x, norm_y))

    #     for area in self.areas:
    #         while area["geometry"].contains(point):
    #             area["callback"]()
    #             point = 0

        
    # Thread(target=click_event).start()
                

    def stop(self):
        '''Stop command'''
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.move_publisher.publish(msg)
        time.sleep(0.20)
        msg.linear.x = 0
        self.move_publisher.publish(msg)

        
    def go_forwards(self):
        '''Forward command'''
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

        # section that allows for sound to play without making it too repetitive 
        while self.timer_forward < .1:
            self.timer_forward += .1
            self.forward_sound()
            time.sleep(.1)

        self.timer_forward += .1

        if self.timer_forward > 1.5:
            self.timer_forward = 0 

        print("move_forward")


    def go_backwards(self):
        '''Backwards command'''
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

        # section that allows for sound to play without making it too repetitive 
        while self.timer_back < .1:
            self.timer_back += .1
            self.backwards_sound()
            time.sleep(.1)

        self.timer_back += .1

        if self.timer_back > 1.5:
            self.timer_back = 0 

        print("move_backwards")    

    def turn_left(self):
        '''Turn left command'''
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

        # section that allows for sound to play without making it too repetitive 
        while self.timer_left_turn < .1:
            self.timer_left_turn += .1
            self.left_sound()
            time.sleep(.1)

        self.timer_left_turn += .1

        if self.timer_left_turn > 1.5:
            self.timer_left_turn = 0 

        print("move_left")        

    def turn_right(self):
        '''Turn right command'''
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

        # section that allows for sound to play without making it too repetitive 
        while self.timer_right_turn < .1:
            self.timer_right_turn += .1
            self.right_sound()
            time.sleep(.1)

        self.timer_right_turn += .1

        if self.timer_right_turn > 1.5:
            self.timer_right_turn = 0 

        print("move_right")        

    # Sound commands
    def forward_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/forward.mp3')

    def backwards_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/back.mp3')

    def left_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/left_turn.mp3')

    def right_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/right_turn.mp3')

class ArmController():
    '''Controls for the arm! (NOT GRIPPER)'''
    def __init__(self, forwards_scale = 1, backwards_scale = 1, left_scale = 1, right_scale = 1, parent=None):
        self.areas = [
            # This actually extends
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (1, 0)]),
                "callback" : self.move_up
            },
            {
                "geometry" : shapely.Polygon([(0, 1), (0.5, 0.5), (1, 1)]),
                "callback" : self.move_down
            },
            {
                "geometry" : shapely.Polygon([(0, 0), (0.5, 0.5), (0, 1)]),
                "callback" : self.extend
            },
            {
                "geometry" : shapely.Polygon([(1, 0), (0.5, 0.5), (1, 1)]),
                "callback" : self.retract
            }
        ]
        self.timer_up = 0
        self.timer_down = 0
        self.timer_in = 0
        self.timer_out = 0
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
        '''Detects when the mouse clicks on the screen'''
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
        

    def move_up(self):
        '''Arm move up command'''
        command = {'joint': 'joint_lift', 'delta': self.delt_vert}
        self.send_command(command)

        # section that allows for sound to play without making it too repetitive 
        while self.timer_up < .1:
            self.timer_up += .1
            self.up_sound()
            time.sleep(.1)

        self.timer_up += .1

        if self.timer_up > 1.5:
            self.timer_up = 0 

        print("arm_move_up")

    def move_down(self):
        '''Arm move down command'''
        command = {'joint': 'joint_lift', 'delta': -self.delt_vert}
        self.send_command(command)

        # section that allows for sound to play without making it too repetitive 
        while self.timer_down < .1:
            self.timer_down += .1
            self.down_sound()
            time.sleep(.1)

        self.timer_down += .1

        if self.timer_down > 1.5:
            self.timer_down = 0 

        print("arm_move_down")

    def extend(self):
        '''Arm extend command'''
        command = {'joint': ['joint_arm_l0','joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'], 'delta': -self.delt_horiz}
        self.send_arm_command(command)

        # section that allows for sound to play without making it too repetitive 
        while self.timer_out < .1:
            self.timer_out += .1
            self.in_sound()
            time.sleep(.1)

        self.timer_out += .1

        if self.timer_out > 1.5:
            self.timer_out = 0 

        print("arm_move_extend")

    def retract(self):
        '''Arm retract command'''
        command = {'joint': ['joint_arm_l0','joint_arm_l1', 'joint_arm_l2', 'joint_arm_l3'], 'delta': self.delt_horiz}
        self.send_arm_command(command)

        # section that allows for sound to play without making it too repetitive 
        while self.timer_in < .1:
            self.timer_in += .1
            self.out_sound()
            time.sleep(.1)

        self.timer_in += .1

        if self.timer_in > 1.5:
            self.timer_in = 0 

        print("arm_retract")

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


    # Sound commands
    def in_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/arm_in.mp3')

    def out_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/arm_out.mp3')

    def up_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/arm_up.mp3')

    def down_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/arm_down.mp3')

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
        # starting positions
        self.timer_close = 0
        self.timer_open = 0
        self.timer_left = 0
        self.timer_right = 0
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
        dimensions = self.parent.image_frame.frameGeometry()
        norm_x = event.x() / dimensions.width()
        norm_y = event.y() / dimensions.height()

        point = shapely.Point((norm_x, norm_y))

        for area in self.areas:
            if area["geometry"].contains(point):
                area["callback"]()
        

    def open_gripper(self):
        """open gripper command"""
        command = {'joint': 'joint_gripper_finger_left', 'delta': self.grip_factor}
        self.send_command(command)

        # section that allows for sound to play without making it too repetitive 
        while self.timer_open < .1:
            self.timer_open += .1
            self.open_sound()
            time.sleep(.1)

        self.timer_open += .1

        if self.timer_open > 1.5:
            self.timer_open = 0 

        print("open_gripper")

    def close_gripper(self):
        """Close gripper command"""
        command = {'joint': 'joint_gripper_finger_left', 'delta': -self.grip_factor}
        self.send_command(command)

        # section that allows for sound to play without making it too repetitive 
        while self.timer_close < .1:
            self.timer_close += .1
            self.close_sound()
            time.sleep(.1)

        self.timer_close += .1

        if self.timer_close > 1.5:
            self.timer_close = 0 

        print("close_gripper")



    def turn_left(self):
        """turn gripper to the left"""
        command = {'joint': 'joint_wrist_yaw', 'delta': self.yaw_factor}
        self.send_command(command)

        # section that allows for sound to play without making it too repetitive 
        while self.timer_left < .1:
            self.timer_left += .1
            self.gripper_left_sound()
            time.sleep(.1)

        self.timer_left += .1

        if self.timer_left > 1.5:
            self.timer_left = 0 

        print("left_gripper")


    def turn_right(self):
        """Turn gripper to the right"""
        command = {'joint': 'joint_wrist_yaw', 'delta': -self.yaw_factor}
        self.send_command(command)

        # section that allows for sound to play without making it too repetitive 
        while self.timer_right < .1:
            self.timer_right += .1
            self.gripper_right_sound()
            time.sleep(.1)

        self.timer_right += .1

        if self.timer_right > 1.5:
            self.timer_right = 0 

        print("right_gripper")

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
        


    # Sound commands
    def close_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/close.mp3')

    def open_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/open.mp3')

    def gripper_left_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/arm_left.mp3')

    def gripper_right_sound(self):
        playsound('/home/hello-robot/catkin_ws/src/joystick_commands/scripts/sounds/arm_right.mp3')

#######################################################################


class SquareButton(QPushButton):
    ''' Creates buttons (this section you select the size and shape)'''
    def __init__(self, parent=None, dimensions=(430, 173)):
        super(QPushButton, self).__init__(parent)
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
        self.left_button = SquareButton(parent=self)
        self.pointSize = 60                                     
        self.fontD = self.font()
        self.fontD.setPointSize(self.pointSize)
        self.left_button.setFont(self.fontD) 
        self.left_button.setText("")
        # using icon allows use to implement images 
        icon = QtGui.QIcon()

        icon.addPixmap(QtGui.QPixmap("/home/hello-robot/catkin_ws/src/joystick_commands/scripts/images/left.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.left_button.setIcon(icon)
        self.left_button.setIconSize(QtCore.QSize(300, 300))
        

        self.right_button = SquareButton(parent=self)
        self.right_button.setFont(self.fontD)
        self.right_button.setText("")
        # using icon allows use to implement images 
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap("/home/hello-robot/catkin_ws/src/joystick_commands/scripts/images/right.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.right_button.setIcon(icon1)
        self.right_button.setIconSize(QtCore.QSize(300, 300))

        self.left_button.clicked.connect(self.controller.turn_left)
        self.right_button.clicked.connect(self.controller.turn_right)

        #This part takes the button you created and add them to QT interface
        self.main_layout = QVBoxLayout()
        self.middle_widget = QWidget()
        self.middle_layout = QHBoxLayout()

        self.middle_layout.addWidget(self.left_button)
        self.middle_layout.addWidget(self.right_button)
        self.middle_layout.setAlignment(self.left_button, Qt.AlignLeft)
        self.middle_layout.setAlignment(self.right_button, Qt.AlignRight)
        self.middle_widget.setLayout(self.middle_layout)

        self.main_layout.addWidget(self.middle_widget)

        self.setLayout(self.main_layout)
        self.setFixedSize(900, 200)

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
        self.up_button.setText("")

        # using icon allows use to implement images 
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap("/home/hello-robot/catkin_ws/src/joystick_commands/scripts/images/open2.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.up_button.setIcon(icon3)
        self.up_button.setIconSize(QtCore.QSize(200, 200))
        
        self.down_button = Button(parent=self)
        self.down_button.setFont(self.fontD)
        self.down_button.setText("")

        # using icon allows use to implement images 
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap("/home/hello-robot/catkin_ws/src/joystick_commands/scripts/images/close.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.down_button.setIcon(icon2)
        self.down_button.setIconSize(QtCore.QSize(200, 200))

        self.up_button.clicked.connect(self.controller.open_gripper)
        self.down_button.clicked.connect(self.controller.close_gripper)

        #This part takes the button creation and made them to QT interface
        self.main_layout = QVBoxLayout()
        self.middle_widget = QWidget()
        self.middle_layout = QHBoxLayout()

        self.main_layout.addWidget(self.up_button)
        self.main_layout.addWidget(self.middle_widget)
        self.main_layout.addWidget(self.down_button)

        self.main_layout.setAlignment(self.up_button, Qt.AlignTop)
        self.main_layout.setAlignment(self.down_button, Qt.AlignBottom)

        self.main_layout.setAlignment(self.up_button, Qt.AlignCenter)
        self.main_layout.setAlignment(self.down_button, Qt.AlignCenter)

        self.setLayout(self.main_layout)
        self.setFixedSize(750, 200)

class DisplayImageWidget(QWidget):
    """allows images(camera feedback) to be added to the correct main display section"""
    def __init__(self, parent=None):
        super(DisplayImageWidget, self).__init__(parent)
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
        """uses the dimensions of the camera layout to draw triangles on the images to divide the sections"""
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
    '''Creates a page call Manipulation and display all previous information such as images, and places them in the correct location'''
    condition = input("")
    print("Session data: " + condition)
    def __init__(self):
        super(QWidget, self).__init__()
        self.layout = QVBoxLayout()

        #####################################################################

        self.arm_widget = QWidget()
        self.arm_layout = QHBoxLayout()
        """Adding labels"""
        self.arm_camera_widget = QWidget()
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap("back3.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.arm_camera_label = QLabel(text="BASE MOVEMENT")
        self.arm_camera_label.setStyleSheet("QLabel{font-size: 15pt;}")

        self.move_label = QLabel(text="Forward")
        self.move_label.setStyleSheet("QLabel{font-size: 10pt;}")

        self.slow_label = QLabel(text="Backward")
        self.slow_label.setStyleSheet("QLabel{font-size: 10pt;}")

        self.labelmove = QLabel(self)
        self.labelmove.setGeometry(QtCore.QRect(360, 900, 171, 61))
        self.labelmove.setPixmap(QtGui.QPixmap("back3.png"))
        self.labelmove.setScaledContents(True)

        # This controls the size of the image shown and what is connected to the clicking for that image
        self.arm_camera = DisplayImageWidget(parent=self)
        self.arm_camera.image_frame.setFixedSize(800, 700)

        # This area is what connect the camera to the position "gripper" will open up the camera that is name in gripper
        self.arm_camera.available_modes["arm"] = {"show_function" : self.arm_camera.show_navigation, "controller" : NavigationController(parent=self.arm_camera)}
        self.arm_camera.set_mode("arm")

        self.arm_camera_layout = QVBoxLayout()

        self.arm_camera_layout.addWidget(self.arm_camera_label)

        self.arm_camera_layout.addWidget(self.arm_camera)
    
        self.cam_buttons = GripperButtonset(parent=self)

        self.arm_camera_layout.setAlignment(self.move_label, Qt.AlignHCenter)
        self.arm_camera_layout.setAlignment(self.slow_label, Qt.AlignHCenter)

        self.arm_camera_layout.addWidget(self.cam_buttons)
        self.arm_camera_layout.setAlignment(self.arm_camera_label, Qt.AlignHCenter)
        self.arm_camera_widget.setLayout(self.arm_camera_layout)
        self.arm_camera_layout.setAlignment(self.cam_buttons, Qt.AlignCenter)

        self.arm_cam_thread = threading.Thread(target=self.run_arm_camera, daemon=True)
        self.arm_cam_thread.start()

        #####################################################################
        # Arm motion/gripper camera view
        self.gripper_widget = QWidget()
        self.gripper_layout = QHBoxLayout()



        self.gripper_camera_widget = QWidget()

        self.gripper_camera_label = QLabel(text="Arm Movement")
        self.gripper_camera_label.setStyleSheet("QLabel{font-size: 15pt;}")

        self.direction_camera_label = QLabel(text="Arm UP")
        self.direction_camera_label.setStyleSheet("QLabel{font-size: 10pt;}")

        # self.gripper_camera_label.setFixedHeight(20)
        self.direction_label = QLabel(text="ARM DOWN")
        self.direction_label.setStyleSheet("QLabel{font-size: 10pt;}")

        self.gripper_camera = DisplayImageWidget(parent=self)
        self.gripper_camera.image_frame.setFixedSize(800, 700)
        
        # This area is what connect the camera to the position "gripper" will open up the camera that is name in gripper
        self.gripper_camera.available_modes["gripper"] = {"show_function" : self.gripper_camera.show_navigation, "controller" : ArmController(parent=self.gripper_camera)}
        self.gripper_camera.set_mode("gripper")
        self.gripper_camera_layout = QVBoxLayout()
        self.gripper_camera_layout.addWidget(self.gripper_camera_label)
        
        # self.gripper_camera_layout.addWidget(self.direction_camera_label)
        self.gripper_camera_layout.addWidget(self.gripper_camera)

        # self.gripper_camera_layout.addWidget(self.direction_label)
        self.gripper_camera_layout.setAlignment(self.direction_label, Qt.AlignCenter)
        self.gripper_camera_layout.setAlignment(self.direction_camera_label, Qt.AlignCenter)
        self.cam_buttonss = Buttonset(parent=self)
        
        self.gripper_camera_layout.addStretch()
        self.gripper_camera_layout.addWidget(self.cam_buttonss)
        self.gripper_camera_layout.setAlignment(self.gripper_camera_label, Qt.AlignCenter)

        self.gripper_camera_widget.setLayout(self.gripper_camera_layout)
        self.gripper_camera_layout.setAlignment(self.cam_buttonss, Qt.AlignCenter)

        self.gripper_cam_thread = threading.Thread(target=self.main_camera_cb, daemon=True)
        self.gripper_cam_thread.start()

        self.arm_layout.addWidget(self.arm_camera_widget)
        self.arm_layout.addWidget(self.gripper_camera_widget)

        self.arm_widget.setLayout(self.arm_layout)

        self.layout.addWidget(self.arm_widget)

        self.layout.addStretch()



        self.setLayout(self.layout)


#######################################################################         
# Detecting cameras
    # camera for gripper       
    def main_camera_cb(self):
        """detects camera in port 8 (which is the gripper camera)"""
        vid = cv2.VideoCapture(8)
        while(True):
            # Capture the video frame
            # by frame
            _, frame = vid.read()
            self.gripper_camera.show_image_by_mode(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
#####################################################################
    # Camera for movement (webcam)
    def run_arm_camera(self):
        """detects camera in port 6 (webcam camera that is connected on the usb port near the speakers)"""
        vid = cv2.VideoCapture(6)
        while(True):
            # Capture the video frame
            # by frame
            _, frame = vid.read()
            self.arm_camera.show_image_by_mode(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))

if __name__=="__main__":
    #logging.basicConfig(format='%(levelname)s %(asctime)s: %(message)s',filename=f"{os.path.dirname(os.path.abspath(__file__))}/logs/qt_interface_{time.strftime('%y_%m_%d:%H_%M_%S', time.localtime(time.time()))}.log", level=logging.INFO, datefmt="%y-%m-%d:%h-%m-%s")
    #logging.info("Starting application")
    rospy.init_node("qt_interface", anonymous=True)

    app = QApplication(sys.argv)
    window = ManipulationPage()

    window.show()

    #logging.info("Application started")

    app.exec()
