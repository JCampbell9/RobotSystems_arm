#!/usr/bin/python3
# coding=utf8
import sys
# sys.path.append('/home/pi/ArmPi/')
sys.path.append('/Users/socce/Desktop/git_repos/RobotSystems_arm/ArmPi/')
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

AK = ArmIK()

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


# The angle at which the gripper is closed when gripping
servo1 = 500

# initial position
def initMove():
    """
    Default position for the arm
    :return:
    """
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)


count = 0
track = False
_stop = False
get_roi = False
center_list = []
first_move = True
__isRunning = False
detect_color = 'None'
action_finish = True
start_pick_up = False
start_count_t1 = True
# Variable reset

def reset():
    """"
    For global variable usage
    """
    global count
    global track
    global _stop
    global get_roi
    global first_move
    global center_list
    global __isRunning
    global detect_color
    global action_finish
    global start_pick_up
    global __target_color
    global start_count_t1

    count = 0
    _stop = False
    track = False
    get_roi = False
    center_list = []
    first_move = True
    __target_color = ()
    detect_color = 'None'
    action_finish = True
    start_pick_up = False
    start_count_t1 = True


# app initialization call
def init():
    print("ColorTracking Init")
    initMove()


# app start play call
def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorTracking Start")


# app stop play call
def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Stop")


# app exit gameplay call
def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorTracking Exit")


rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0
world_x, world_y = 0, 0


# Robot arm moves thread
def move():
    global rect
    global track
    global _stop
    global get_roi
    global unreachable
    global __isRunning
    global detect_color
    global action_finish
    global rotation_angle
    global world_X, world_Y
    global world_x, world_y
    global center_list, count
    global start_pick_up, first_move

    # Different color wood fast placement coordinates (x, y, z)  Goal location
    coordinate = {
        'red': (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5, 1.5),
        'blue': (-15 + 0.5, 0 - 0.5, 1.5),
    }
    while True:
        if __isRunning:
            if first_move and start_pick_up:  # When an object is detected for the first time
                action_finish = False
                set_rgb(detect_color)
                setBuzzer(0.1)
                result = AK.setPitchRangeMoving((world_X, world_Y - 2, 5), -90, -90, 0)  # Do not fill in the running time parameter, adaptive running time
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                time.sleep(result[2] / 1000)  # The third item of the return parameter is time
                start_pick_up = False
                first_move = False
                action_finish = True
            elif not first_move and not unreachable:  # Not the first time an object has been detected
                set_rgb(detect_color)
                if track:  # If it is the tracking stage
                    if not __isRunning:  # Stop and exit flag detection
                        continue
                    AK.setPitchRangeMoving((world_x, world_y - 2, 5), -90, -90, 0, 20)
                    time.sleep(0.02)
                    track = False
                if start_pick_up:  # If the object has not moved for a while, start to grip
                    action_finish = False
                    if not __isRunning:  # Stop and exit flag detection
                        continue
                    Board.setBusServoPulse(1, servo1 - 280, 500)  # Paws open

                    # Calculate the angle that the gripper needs to rotate
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0, 1000)  # lower the altitude
                    time.sleep(2)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1, 500)  # Holder closed
                    time.sleep(1)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000) # Robot arm up
                    time.sleep(1)

                    if not __isRunning:
                        continue
                    # Sort and place different color squares
                    result = AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90,
                                                    -90, 0)
                    time.sleep(result[2] / 1000)

                    if not __isRunning:
                        continue
                    servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving(
                        (coordinate[detect_color][0], coordinate[detect_color][1], coordinate[detect_color][2] + 3),
                        -90, -90, 0, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color]), -90, -90, 0, 1000)
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1 - 200, 500)  # Open the paws, put down the object
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0,
                                           800)
                    time.sleep(0.8)

                    initMove()  # Back to initial position
                    time.sleep(1.5)

                    detect_color = 'None'
                    first_move = True
                    get_roi = False
                    action_finish = True
                    start_pick_up = False
                    set_rgb(detect_color)
                else:
                    time.sleep(0.01)
        else:
            if _stop:
                _stop = False
                Board.setBusServoPulse(1, servo1 - 70, 300)
                time.sleep(0.5)
                Board.setBusServoPulse(2, 500, 500)
                AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
                time.sleep(1.5)
            time.sleep(0.01)



# Run child thread
th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

t1 = 0
roi = ()
last_x, last_y = 0, 0