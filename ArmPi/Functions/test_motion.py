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
    
    while True:
        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame, coordinates, rect = camera_perception.main(frame)
            cv2.imshow('Frame', Frame)
            print(f'\n\n MADE IT    Coordinates:  {coordinates} \n\n')
            camera_motion.main(target_loc=(coordinates[0], coordinates[1], 1.5), goal_loc=goal_coordinates[target_color], rect=rect)


def camera_test():
    my_camera = Camera.Camera()
    my_camera.camera_open()
    img = my_camera.frame
    camera_perception = CameraPerception.Perception()
    i = 0
    while True:

        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame, coordinates, _ = camera_perception.main(frame)
            cv2.imshow('Frame', Frame)
            print(f'\n Loop: {i}  Coordinates:  {coordinates} \n')
            key = cv2.waitKey(1)
            if key == 27:
                break
        #print(f'\n loop: {i}  Coordinates:  {coordinates} \n')
        i += 1
    my_camera.camera_close()
    cv2.destroyAllWindows()


if __name__ == '__main__':

    test_motion()
    #camera_test()
