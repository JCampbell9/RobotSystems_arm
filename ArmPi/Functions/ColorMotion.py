#!/usr/bin/python3
# coding=utf8
import sys
sys.path.append('/home/pi/ArmPi/')
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


# The angle at which the gripper is closed when gripping
servo1 = 500
# initial position
def initMove():
    Board.setBusServoPulse(1, servo1 - 50, 300)
    Board.setBusServoPulse(2, 500, 500)
    AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)



count = 0
_stop = False
color_list = []
get_roi = False
__isRunning = False
move_square = False
detect_color = 'None'
start_pick_up = False
start_count_t1 = True

# Place coordinates
coordinate = {
    'red':   (-15 + 1, -7 - 0.5, 1.5),
    'green': (-15 + 1, -7 - 0.5, 1.5),
    'blue':  (-15 + 1, -7 - 0.5, 1.5),
}

z_r = coordinate['red'][2]
z_g = coordinate['green'][2]
z_b = coordinate['blue'][2]
z = z_r


def reset():
    global _stop
    global count
    global get_roi
    global color_list
    global move_square
    global detect_color
    global start_pick_up
    global start_count_t1
    global z_r, z_g, z_b, z

    count = 0
    _stop = False
    color_list = []
    get_roi = False
    move_square = False
    detect_color = 'None'
    start_pick_up = False
    start_count_t1 = True
    z_r = coordinate['red'][2]
    z_g = coordinate['green'][2]
    z_b = coordinate['blue'][2]
    z = z_r


def init():
    """

    :return: none
    """
    print("ColorPalletizing Init")
    initMove()


def start():
    global __isRunning
    reset()
    __isRunning = True
    print("ColorPalletizing Start")


def stop():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorPalletizing Stop")


def exit():
    global _stop
    global __isRunning
    _stop = True
    __isRunning = False
    print("ColorPalletizing Exit")


rect = None
size = (640, 480)
rotation_angle = 0
unreachable = False
world_X, world_Y = 0, 0


def move():
    global rect
    global _stop
    global get_roi
    global move_square
    global __isRunning
    global unreachable
    global detect_color
    global start_pick_up
    global rotation_angle
    global world_X, world_Y
    global z_r, z_g, z_b, z

    dz = 2.5

    while True:
        if __isRunning:
            if detect_color != 'None' and start_pick_up:  # If it is detected that the square has not moved for a period of time, start to grip
                set_rgb(detect_color)
                setBuzzer(0.1)

                # Height accumulation
                z = z_r
                z_r += dz
                if z == 2 * dz + coordinate['red'][2]:
                    z_r = coordinate['red'][2]
                if z == coordinate['red'][2]:
                    move_square = True
                    time.sleep(3)
                    move_square = False
                result = AK.setPitchRangeMoving((world_X, world_Y, 7), -90, -90, 0)  # Move to the target position, height 5cm
                if result == False:
                    unreachable = True
                else:
                    unreachable = False
                    time.sleep(result[2] / 1000)

                    if not __isRunning:
                        continue
                    # Calculate the angle that the gripper needs to rotate
                    servo2_angle = getAngle(world_X, world_Y, rotation_angle)
                    Board.setBusServoPulse(1, servo1 - 280, 500)  # Paws open
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((world_X, world_Y, 2), -90, -90, 0, 1000)  # Lower the height to 2cm
                    time.sleep(1.5)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1, 500)  # Holder closed
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(2, 500, 500)
                    AK.setPitchRangeMoving((world_X, world_Y, 12), -90, -90, 0, 1000)  # Robot arm up
                    time.sleep(1)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0,
                                           1500)
                    time.sleep(1.5)

                    if not __isRunning:
                        continue
                    servo2_angle = getAngle(coordinate[detect_color][0], coordinate[detect_color][1], -90)
                    Board.setBusServoPulse(2, servo2_angle, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], z + 3), -90, -90,
                                           0, 500)
                    time.sleep(0.5)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], z), -90, -90, 0,
                                           1000)
                    time.sleep(0.8)

                    if not __isRunning:
                        continue
                    Board.setBusServoPulse(1, servo1 - 200, 500)  # Open the paw, put down the object
                    time.sleep(1)

                    if not __isRunning:
                        continue
                    AK.setPitchRangeMoving((coordinate[detect_color][0], coordinate[detect_color][1], 12), -90, -90, 0,
                                           800)
                    time.sleep(0.8)

                    initMove()  # Back to initial position
                    time.sleep(1.5)

                    detect_color = 'None'
                    get_roi = False
                    start_pick_up = False
                    set_rgb(detect_color)
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
center_list = []
last_x, last_y = 0, 0
draw_color = range_rgb["black"]