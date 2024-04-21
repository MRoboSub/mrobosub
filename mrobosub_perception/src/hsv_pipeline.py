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
    histogram_equalization: bool = False
    
    erode_radius: int = 2
    dilate_radius: int = 2
    median_radius: int = 4
    guassian_radius: int = 4

    dougpeck_ratio_buoy: float = 0.01
    dougpeck_ratio_pathmarker: float = 0.02

    @dataclass
    class Detection:
        x: int
        y: int
        radius: int
    
    def color_enhancement(self, frame):
        if self.white_balance:
            enhanced = self._simple_white_balance(frame, self.wb_shift, self.wb_scale)
        else:
            enhanced = frame

        if self.histogram_equalization:
            enhanced[:,:,0] = cv2.equalizeHist(enhanced[:,:,0])
            enhanced[:,:,1] = cv2.equalizeHist(enhanced[:,:,1])
            enhanced[:,:,2] = cv2.equalizeHist(enhanced[:,:,2])

        return enhanced

    def lengthBetweenPoint(centerA, centerB):
        return np.sqrt((centerA[0] - centerB[0])**2 + (centerA[1] - centerB[1])**2)

    def calcIntersectionPoints(point1, point2, point3, point4):
        a1 = point2[1] - point1[1]
        b1 = point1[0] - point2[0]
        c1 = a1 * (point1[0]) + b1 * (point1[1])

        a2 = point4[1] - point3[1]
        b2 = point3[0] - point4[0]
        c2 = a2 * (point3[0]) + b2 * (point3[1])

        determinant = a1 * b2 - a2 * b1

        if determinant == 0:
            return (float('inf'), float('inf'))
        else:
            x = (b2 * c1 - b1 * c2) / determinant
            y = (a1 * c2 - a2 * c1) / determinant
            return (int(x), int(y))

    def filter_image(self, frame, color_space=cv2.COLOR_BGR2HSV):
        color_enhancement(frame)
        frame_hsv = cv2.cvtColor(enhanced, color_space)

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


    def find_pathmarker_object(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if (selected_contour := self._get_pathmarker_object(contours)) is not None:
            hull = cv2.convexHull(selected_contour)
            rect = cv2.minAreaRect(hull)
            box = cv2.boxPoints(rect)
            box = np.intp(box)

            #cv2.drawContours(enhanced, [box], 0, (0, 255, 0), 2)

            epsilon = dougpeck_ratio_pathmarker * cv2.arcLength(box, True)
            approxCurve = cv2.approxPolyDP(box, epsilon, True)

            points = approxCurve.reshape(-1, 2)
            
            points = sorted(points, key=lambda x: x[0])

            yaw_angle = 0

            if len(points) == 4:
                left_points = sorted(points[:2], key=lambda x: x[1])
                right_points = sorted(points[2:], key=lambda x: x[1])

                corner_topLeft, corner_botLeft = left_points
                corner_topRight, corner_botRight = right_points

                corner_topLeft = tuple(corner_topLeft)
                corner_topRight = tuple(corner_topRight)
                corner_botLeft = tuple(corner_botLeft)
                corner_botRight = tuple(corner_botRight)

                topSideCenter = lineCenter(corner_topLeft, corner_topRight)
                botSideCenter = lineCenter(corner_botLeft, corner_botRight)
                leftSideCenter = lineCenter(corner_topLeft, corner_botLeft)
                rightSideCenter = lineCenter(corner_topRight, corner_botRight)

                rectCenter = calcIntersectionPoints(leftSideCenter, rightSideCenter, topSideCenter, botSideCenter)
                
                rectCenter = tuple(int(val) for val in rectCenter)
                topSideCenter = tuple(int(val) for val in topSideCenter)
                rightSideCenter = tuple(int(val) for val in rightSideCenter)
                leftSideCenter = tuple(int(val) for val in leftSideCenter)
                botSideCenter = tuple(int(val) for val in botSideCenter)
                
                top_line = lengthBetweenPoint(corner_topLeft, corner_topRight)
                bot_line = lengthBetweenPoint(corner_botLeft, corner_botRight)
                left_line = lengthBetweenPoint(corner_topLeft, corner_botLeft)
                right_line = lengthBetweenPoint(corner_topRight, corner_botRight)
                diff_top_bot = abs(top_line - bot_line)
                diff_left_right = abs(left_line - right_line)

                if lengthBetweenPoint(corner_topLeft, corner_topRight) > lengthBetweenPoint(corner_topLeft, corner_botLeft):
                    center_line = lengthBetweenPoint(topSideCenter, rectCenter);
                    yaw_angle = np.degrees(np.arcsin((topSideCenter[0] - rectCenter[0]) / center_line));
                else:
                    center_line = lengthBetweenPoint(rightSideCenter, rectCenter);
                    yaw_angle = np.degrees(np.arcsin((rightSideCenter[0] - rectCenter[0]) / center_line));

                centroid = rectCenter

            else:
                M = cv2.moments(hull)

                if M["m00"] != 0:
                    centroid_x = int(M["m10"] / M["m00"])
                    centroid_y = int(M["m01"] / M["m00"])
                    centroid = (centroid_x, centroid_y)
                else:
                    contourCentroid = None

            return HsvPipeline.Detection(centroid[0], centroid[1], yaw_angle)
        return None

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

    def _get_pathmarker_object(contours):
        maxHeight = 0
        maxWidth = 0
        counter = -1

        for i, contour in enumerate(contours):
            epsilon = dougpeck_ratio_pathmarker * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)

            x, y, w, h = cv2.boundingRect(contour)

            if h > maxHeight and w > maxWidth and len(approx) >= 4 and len(approx) <= 8 and h > CONTOUR_MIN_HEIGHT and w > CONTOUR_MIN_WIDTH:
                maxHeight = h
                maxWidth = w
                counter = i

        return counter

    def _get_circular_object(self, contours):
        maxHeight = 0
        maxHwRatio = 0
        maxAreaRatio = 0
        selected_contour = None

        for contour in contours:
            hull = cv2.convexHull(contour)

            epsilon = self.dougpeck_ratio_buoy * cv2.arcLength(hull, True)
            approx = cv2.approxPolyDP(hull, epsilon, True)

            x, y, w, h = cv2.boundingRect(hull)
            
            contour_area = cv2.contourArea(hull)
            hw_ratio = 1 - (abs(h - w) / max(h, w))
            cir_radius = max(h, w) / 2
            cir_area = np.pi * np.power(cir_radius, 2)
            area_ratio =  1 - (abs(contour_area - cir_area) / max(contour_area, cir_area))
            
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
