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

# AK = ArmIK()


class Perception:

    def __init__(self, init_image, target_color=('red',)):

        self.target_color = target_color
        self.range_rgb = {
            'red': (0, 0, 255),
            'blue': (255, 0, 0),
            'green': (0, 255, 0),
            'black': (0, 0, 0),
            'white': (255, 255, 255)}
        
        self.roi
        self.rect
        self.world_x
        self.world_y
        self.world_X
        self.world_Y
        self.box

        self.count = 0
        self._stop = False
        self.track = False
        self.get_roi = False
        self.center_list = []
        self.first_move = True
        self.__target_color = ()
        self.detect_color = 'None'
        self.action_finish = True
        self.start_pick_up = False
        self.start_count_t1 = True
        
        self.img = init_image
        self.img_copy = self.img.copy()
        self.img_dim = self.img.shape[:2]
        self.block_coordinates = (0, 0)

    def main(self, img):
        
        self.img = img
        self.img_copy = self.img.copy()
        self.img_dim = self.img.shape[:2]

        self.crosshairs()
        frame_gb = self.frame_resize()

        if self.get_roi and self.start_pick_up:
            self.get_roi = False
            frame_gb = getMaskROI(frame_gb, self.roi, size)

        frame_lab = self.convert_to_lab(frame_gb)

        for i in color_range:
            if i in self.target_color:
                detect_color = i
                frame_mask = self.mask(frame_lab, detect_color=detect_color)

                contours = self.get_contours(frame_mask)
                area_max_contour, area_max= self.get_area_max_contour(contours)
        
        if area_max > 2500:

            box = self.find_block(area_max_contour=area_max_contour)
            self.get_block_location()
            self.draw_roi_indicators(detect_color, box)

        return self.img

    
    def get_contours(self, frame_mask):
        
        opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((6, 6), np.uint8))  # Open operation
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((6, 6), np.uint8))  # Closed operation
        contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # Find the outline

        return contours

    def frame_resize(self):
        
        frame_resize = cv2.resize(self.img_copy, size, interpolation=cv2.INTER_NEAREST)
        frame_gb = cv2.GaussianBlur(frame_resize, (11, 11), 11)
        return frame_gb
    
    def get_area_max_contour(self, contours):

        contour_area_temp = 0
        contour_area_max = 0
        area_max_contour = None

        for c in contours:  # Traverse all contours
            contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate the contour area
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                if contour_area_temp > 300:  # Only when the area is greater than 300, the contour of the largest area is effective to filter interference
                    area_max_contour = c

        return area_max_contour, contour_area_max  # Return the largest contour


    def crosshairs(self):
        
        cv2.line(self.img, (0, int(self.img_dim[0] / 2)), (self.img_dim[1], int(self.img_dim[0] / 2)), (0, 0, 200), 1)
        cv2.line(self.img, (int(self.img_dim[1] / 2), 0), (int(self.img_dim[1] / 2), self.img_dim[0]), (0, 0, 200), 1)


    def convert_to_lab(self, frame_gb):

        return cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)


    def mask(self, frame_lab, detect_color):
        frame_mask = cv2.inRange(frame_lab, color_range[detect_color][0], color_range[detect_color][1])
        return frame_mask
    
    
    def find_block(self, area_max_contour):
        self.rect = cv2.minAreaRect(area_max_contour)
        box = np.int0(cv2.boxPoints(self.rect))

        self.roi = getROI(box)  # Get roi area
        self.get_roi = True
        
        return box
    
    
    def get_block_location(self):

        img_centerx, img_centery = getCenter(self.rect, self.roi, size, square_length)  # Get the center coordinates of the block
        self.world_x, self.world_y = convertCoordinate(img_centerx, img_centery, size)  # Convert to real world coordinates


    def draw_roi_indicators(self, detect_color, box):


        cv2.drawContours(self.img, [box], -1, range_rgb[detect_color], 2)
        cv2.putText(self.img, '(' + str(self.world_x) + ',' + str(self.world_y) + ')', (min(box[0, 0], box[2, 0]), box[2, 1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, range_rgb[detect_color], 1)  # Draw center point




if __name__ == '__main__':

    # init()
    # start()
    my_camera = Camera.Camera()
    my_camera.camera_open()
    img = my_camera.frame
    camera_perception = Perception(img)

    while True:

        img = my_camera.frame
        if img is not None:
            frame = img.copy()
            Frame = camera_perception.main(frame)
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
    my_camera.camera_close()
    cv2.destroyAllWindows()
