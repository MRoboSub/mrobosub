from __future__ import annotations

from dataclasses import dataclass
from itertools import count
from typing import Union

import cv2
import numpy as np


@dataclass
class HsvPipeline():
    hue_lo: int
    hue_hi: int
    sat_lo: int
    sat_hi: int
    val_lo: int
    val_hi: int

    contour_min_height: int = 5
    contour_min_width: int = 5
    
    height_width_ratio_thresh: float = 0.4
    area_container_perim_ratio_thresh: float = 0.4

    wb_shift: int = 200
    wb_scale: int = 120
    white_balance: bool = False
    
    erode_radius: int = 2
    dilate_radius: int = 2
    median_radius: int = 4
    guassian_radius: int = 4

    dougpeck_ratio: float = 0.01

    @dataclass
    class Detection:
        x: int
        y: int
        radius: int
    
    def filter_image(self, frame):
        if self.white_balance:
            enhanced = self._simple_white_balance(frame, self.wb_shift, self.wb_scale)
        else:
            enhanced = frame

        frame_hsv = cv2.cvtColor(enhanced, cv2.COLOR_BGR2HSV)

        if self.median_radius > 0:
            frame_hsv = cv2.medianBlur(frame_hsv, ksize=self.median_radius*2+1)
        if self.guassian_radius > 0:
            frame_hsv = cv2.GaussianBlur(frame_hsv, (self.guassian_radius*2 + 1, self.guassian_radius*2 + 1), 0)

        lower_bound = np.array([self.hue_lo, self.sat_lo, self.val_lo])
        upper_bound = np.array([self.hue_hi, self.sat_hi, self.val_hi])
        
        mask = cv2.inRange(frame_hsv, lower_bound, upper_bound)
        mask = cv2.equalizeHist(mask)

        if self.erode_radius > 0:
            mask = cv2.erode(mask, np.ones((self.erode_radius, self.erode_radius), np.uint8), iterations=1)
        if self.dilate_radius > 0:
            mask = cv2.dilate(mask, np.ones((self.dilate_radius, self.dilate_radius), np.uint8), iterations=1)

        return mask


    def find_circular_object(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if (selected_contour := self._get_circular_object(contours)) is not None:
            hull = cv2.convexHull(selected_contour)
            # hull_points = [cv2.convexHull(contour) for contour in [selected_contour]]

            M = cv2.moments(hull)

            if M["m00"] != 0:
                centroid_x = int(M["m10"] / M["m00"])
                centroid_y = int(M["m01"] / M["m00"])
                centroid = (centroid_x, centroid_y)
            else:
                return None
            
            x, y, w, h = cv2.boundingRect(hull)
            contour_area = cv2.contourArea(hull)

            radius = max(h, w) / 2
            area = np.pi * np.power(radius, 2)

            return HsvPipeline.Detection(centroid_x, centroid_y, radius)

        return None
 
    def _simple_white_balance(self, mat, wb_shift, wb_scale):
        result = cv2.cvtColor(mat, cv2.COLOR_BGR2LAB)
        avg_a = np.average(result[:, :, 1])
        avg_b = np.average(result[:, :, 2])
        result[:, :, 1] = result[:, :, 1] - ((avg_a - wb_shift) * (result[:, :, 0] / 255.0) * (wb_scale/100))
        result[:, :, 2] = result[:, :, 2] - ((avg_b - wb_shift) * (result[:, :, 0] / 255.0) * (wb_scale/100))
        result = cv2.cvtColor(result, cv2.COLOR_LAB2BGR)
        return result
    
    def _get_circular_object(self, contours):
        maxHeight = 0
        maxHwRatio = 0
        maxAreaRatio = 0
        selected_contour = None

        for contour in contours:
            hull = cv2.convexHull(contour)

            epsilon = self.dougpeck_ratio * cv2.arcLength(hull, True)
            approx = cv2.approxPolyDP(hull, epsilon, True)

            x, y, w, h = cv2.boundingRect(hull)
            
            contour_area = cv2.contourArea(hull)
            hw_ratio = 1 - (abs(h - w) / max(h, w))
            cir_radius = max(h, w) / 2
            cir_area = np.pi * np.power(cir_radius, 2)
            area_ratio =  1 - (abs(contour_area - cir_area) / max(contour_area, cir_area))
            
            #print("Curve ={:.2f}".format(len(approx)))

            if hw_ratio > maxHwRatio and area_ratio > maxAreaRatio and h > maxHeight and \
                  hw_ratio > self.height_width_ratio_thresh and area_ratio > self.area_container_perim_ratio_thresh and \
                  len(approx) >= 5 and len(approx) <= 13 and h > self.contour_min_height and w > self.contour_min_width:
                maxHeight = h
                maxHwRatio = hw_ratio
                maxAreaRatio = area_ratio
                selected_contour = contour

        return selected_contour



if __name__ == '__main__':
    img = cv2.imread('frame0138.jpg')
    pipeline = HsvPipeline(109-10, 109+10, 250-50, 250+50, 138-90, 138+90)
    mask = pipeline.filter_image(img)
    cv2.imwrite("mask.jpg", mask)
    detection = pipeline.find_circular_object(mask)
    cv2.circle(img, (detection.x,detection.y), int(detection.radius), (255,0,0))
    cv2.imshow("Image", img)
    cv2.waitKey(-1)
    print(detection)