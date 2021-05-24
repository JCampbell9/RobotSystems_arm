#!/usr/bin/python3
# coding=utf8


import Camera_Motion
import CameraPerception

import sys
sys.path.append('/home/pi/ArmPi/')
import cv2
import time
import Camera

def test_motion(target_color='red'):
    goal_coordinates = {
        'red': (-15 + 0.5, 12 - 0.5, 1.5),
        'green': (-15 + 0.5, 6 - 0.5, 1.5),
        'blue': (-15 + 0.5, 0 - 0.5, 1.5),
        'stacking': (-15 + 1, -7 - 0.5, 1.5)
    }

    my_camera = Camera.Camera()
    my_camera.camera_open()
    img = my_camera.frame
    camera_perception = CameraPerception.Perception()
    camera_motion = Camera_Motion.MoveBlock()

    if img is not None:
        frame = img.copy()
        Frame, coordinates, rect = camera_perception.main(frame)
        cv2.imshow('Frame', Frame)
        camera_motion.main(target_loc=(coordinates[0], coordinates[1], 2), goal_loc=goal_coordinates[target_color], rect=rect)