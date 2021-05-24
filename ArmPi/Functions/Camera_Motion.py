#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
# sys.path.append('/Users/socce/Desktop/git_repos/RobotSystems_arm/ArmPi/')
import cv2
import time
import Camera
import threading
from LABConfig import *
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
import HiwonderSDK.Board as Board
from CameraCalibration.CalibrationConfig import *

if sys.version_info.major == 2:
    print('Please run this program with python3!')
    sys.exit(0)


class MoveBlock:

    def __init__(self):
        self.AK = ArmIK()

        self.gripper_close = 500

        self.goal_coordinates = {
            'red': (-15 + 0.5, 12 - 0.5, 1.5),
            'green': (-15 + 0.5, 6 - 0.5, 1.5),
            'blue': (-15 + 0.5, 0 - 0.5, 1.5),
            'stacking': (-15 + 1, -7 - 0.5, 1.5)
        }

    def main(self, target_loc, goal_loc, rect):

        self.init_pose()

        # move above the target block
        if not self.move_arm((target_loc[0], target_loc[1], target_loc[2]+3), time_delay=False):
            print("target location is unreachable")
            return False
        # get the gripper ready to pick up block
        self.open_gripper()
        self.angle_gripper((target_loc[0], target_loc[1], rect[2]))
        # Grab the block
        self.move_arm(target_loc, time_delay=1.5)
        self.close_gripper()
        # lift block up
        self.move_arm((target_loc[0], target_loc[1], target_loc[2] + 10), time_delay=1)
        # move block above goal location
        self.move_arm((goal_loc[0], goal_loc[1], goal_loc[2] + 8), time_delay=False)
        # properly angle gripper
        self.angle_gripper((goal_loc[0], goal_loc[1], -90))
        # lower the gripper to 3cm above the final z
        self.move_arm((goal_loc[0], goal_loc[1], goal_loc[2] + 3), time_delay=1)
        # set block down
        self.move_arm(goal_loc, time_delay=0.5)
        self.open_gripper()
        self.move_arm((goal_loc[0], goal_loc[1], goal_loc[2] + 10), time_delay=0.8)
        self.init_pose()

    def init_pose(self):
        Board.setBusServoPulse(1, self.gripper_close - 50, 300)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(1.5)

    def open_gripper(self):
        Board.setBusServoPulse(1, self.gripper_close - 280, 500)  # Paws open
        time.sleep(0.5)

    def angle_gripper(self, loc):
        gripper_angle = getAngle(loc)
        Board.setBusServoPulse(2, gripper_angle, 500)
        time.sleep(0.8)

    def close_gripper(self):
        Board.setBusServoPulse(1, self.gripper_close, 500)  # Paws open
        time.sleep(0.8)

    def move_arm(self, target_loc, time_delay):

        if not time_delay:
            result = self.AK.setPitchRangeMoving(target_loc, -90, -90, 0)
            if not result:
                return False
            time.sleep(result[2] / 1000)
        else:
            result = self.AK.setPitchRangeMoving(target_loc, -90, -90, 0, time_delay * 1000)
            if not result:
                return False
            time.sleep(time_delay)


