import cv2
import numpy
import math
from enum import Enum

# Mergeing line code taken from
# https://stackoverflow.com/questions/45531074/how-to-merge-lines-after-houghlinesp

class PathmarkerPipeline:
    """
    An OpenCV pipeline generated by GRIP.
    """

    def __init__(self):
        """initializes all values to presets or None if need to be set
        """
        self.source = None

        self._blur_type = BlurType.Box_Blur
        self._blur_radius = 5.148004411577104

        self.blur_output = None

        self._hsv_threshold_input = self.blur_output
        self._hsv_threshold_hue = [0.0, 45]
        self._hsv_threshold_saturation = [46, 151]
        self._hsv_threshold_value = [63, 255.0]

        self.hsv_threshold_output = None

        self._find_lines_input = self.hsv_threshold_output

        self.find_lines_output = None

        self._filter_lines_lines = self.find_lines_output
        self._filter_lines_min_length = 100.0
        self._filter_lines_angle = [0, 360]

        self.filter_lines_output = None

        self.angle1 = 0.0
        self.angle2 = 0.0
        self.angle = 0.0
        self.found = False
        self.max_diff = ((0,0), 0,0)


    def process(self, source0):
        """
        Runs the pipeline and sets all outputs to new values.
        """
        self.source = source0
        # Step Blur0:
        self._blur_input = source0
        (self.blur_output) = self._blur(self._blur_input, self._blur_type, self._blur_radius)

        # Step HSV_Threshold0:
        self._hsv_threshold_input = self.blur_output
        (self.hsv_threshold_output) = self._hsv_threshold(self._hsv_threshold_input, self._hsv_threshold_hue, self._hsv_threshold_saturation, self._hsv_threshold_value)
        # Step Find_Lines0:
        self._find_lines_input = self.hsv_threshold_output
        (self.find_lines_output) = self._find_lines(self._find_lines_input)

        _lines_x = []
        _lines_y = []
        # print(self.find_lines_output)
        for line_i in self.find_lines_output:
            orientation_i = math.atan2((line_i[0][0]-line_i[0][1]),(line_i[1][0]-line_i[1][1]))
            if (abs(math.degrees(orientation_i)) > 45) and abs(math.degrees(orientation_i)) < (90+45):
                _lines_y.append(line_i)
            else:
                _lines_x.append(line_i)

        _lines_x = sorted(_lines_x, key=lambda _line: _line[0][0])
        _lines_y = sorted(_lines_y, key=lambda _line: _line[0][1])

        merged_lines_x = merge_lines_pipeline_2(_lines_x)
        merged_lines_y = merge_lines_pipeline_2(_lines_y)

        merged_lines_all = []
        merged_lines_all.extend(merged_lines_x)
        merged_lines_all.extend(merged_lines_y)


        # Step Filter_Lines0:
        self._filter_lines_lines = self.find_lines_output
        (self.filter_lines_output) = self._filter_lines(self._filter_lines_lines, self._filter_lines_min_length, self._filter_lines_angle)

        # for line in self.filter_lines_output:
        #     cv2.line(source0, (line.x1, line.y1), (line.x2, line.y2), (255,0,0))
        # cv2.imshow("lines", source0)

        if len(self.filter_lines_output) >= 2:
            # # differences = []
            # # differences.append(((0, 1), abs(self.filter_lines_output[0].angle() - self.filter_lines_output[1].angle())))
            # # differences.append(((0, 2), abs(self.filter_lines_output[0].angle() - self.filter_lines_output[2].angle())))
            # # differences.append(((1, 2), abs(self.filter_lines_output[1].angle() - self.filter_lines_output[2].angle())))

            # self.max_diff = max(differences, key=lambda x: x[1])[0]

            self.angle1 = self.filter_lines_output[0].angle()
            # first = self.filter_lines_output[self.max_diff[0]]
            # print("(" + str(first.x1) + ", " + str(first.y1) + ") to (" + str(first.x2) + ", " + str(first.y2) + ")")

            # second = self.filter_lines_output[self.max_diff[1]]
            # print("(" + str(second.x1) + ", " + str(second.y1) + ") to (" + str(second.x2) + ", " + str(second.y2) + ")")

            self.angle2 = self.filter_lines_output[1].angle()

            if (self.angle1 - self.angle2 > 90):
                self.angle2 = self.angle2 - 180

            elif (self.angle2 - self.angle1 > 90):
                self.angle1 = self.angle1 - 180

            if (abs(self.angle1 - self.angle2) > 20.0):
                self.angle = 0.0
                self.found = False
                # print("extremele angle difference detected")
                return

            self.angle = (self.angle1 + self.angle2) / 2.0
            self.found = True
            # print("self.angle: , ", self.angle)
            # print("self.angle1: , ", self.angle1)
            # print("self.angle2: , ", self.angle2)
        else:
            self.angle = 0.0
            self.found = False


    @staticmethod
    def _blur(src, type, radius):
        """Softens an image using one of several filters.
        Args:
            src: The source mat (numpy.ndarray).
            type: The blurType to perform represented as an int.
            radius: The radius for the blur as a float.
        Returns:
            A numpy.ndarray that has been blurred.
        """
        if(type is BlurType.Box_Blur):
            ksize = int(2 * round(radius) + 1)
            return cv2.blur(src, (ksize, ksize))
        elif(type is BlurType.Gaussian_Blur):
            ksize = int(6 * round(radius) + 1)
            return cv2.GaussianBlur(src, (ksize, ksize), round(radius))
        elif(type is BlurType.Median_Filter):
            ksize = int(2 * round(radius) + 1)
            return cv2.medianBlur(src, ksize)
        else:
            return cv2.bilateralFilter(src, -1, round(radius), round(radius))

    @staticmethod
    def _hsv_threshold(input, hue, sat, val):
        """Segment an image based on hue, saturation, and value ranges.
        Args:
            input: A BGR numpy.ndarray.
            hue: A list of two numbers the are the min and max hue.
            sat: A list of two numbers the are the min and max saturation.
            lum: A list of two numbers the are the min and max value.
        Returns:
            A black and white numpy.ndarray.
        """
        out = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
        return cv2.inRange(out, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))

    class Line:

        def __init__(self, x1, y1, x2, y2):
            self.x1 = x1
            self.y1 = y1
            self.x2 = x2
            self.y2 = y2

        def length(self):
            return numpy.sqrt(pow(self.x2 - self.x1, 2) + pow(self.y2 - self.y1, 2))

        def angle(self):
            if self.y2 > self.y1:
                return math.degrees(math.atan2(self.y2 - self.y1, self.x2 - self.x1))
            else:
                return math.degrees(math.atan2(self.y1 - self.y2, self.x1 - self.x2))
    @staticmethod
    def _find_lines(input):
        """Finds all line segments in an image.
        Args:
            input: A numpy.ndarray.
        Returns:
            A filtered list of Lines.
        """
        detector = cv2.createLineSegmentDetector()
        if (len(input.shape) == 2 or input.shape[2] == 1):
            lines = detector.detect(input)
        else:
            tmp = cv2.cvtColor(input, cv2.COLOR_BGR2GRAY)
            lines = detector.detect(tmp)
        output = []
        if len(lines) != 0 and type(lines[0]) != type(None):
            for i in range(1, len(lines[0])):
                new_tmp = [[int(lines[0][i, 0][0]), int(lines[0][i, 0][1])], [int(lines[0][i, 0][2]), int(lines[0][i, 0][3])]]
                tmp = PathmarkerPipeline.Line(lines[0][i, 0][0], lines[0][i, 0][1],
                                lines[0][i, 0][2], lines[0][i, 0][3])
                output.append(new_tmp)
        return output

    @staticmethod
    def _filter_lines(inputs, min_length, angle):
        """Filters out lines that do not meet certain criteria.
        Args:
            inputs: A list of Lines.
            min_Lenght: The minimum lenght that will be kept.
            angle: The minimum and maximum angles in degrees as a list of two numbers.
        Returns:
            A filtered list of Lines.
        """
        lines = []
        outputs = []
        for i in inputs:
            tmp = PathmarkerPipeline.Line(i[0][0], i[0][1],
                            i[1][0], i[1][1])
            lines.append(tmp)
        lines.sort(key=lambda x: x.length())
        if len(lines) > 2:
            outputs.append(lines[-1])
            outputs.append(lines[-2])
            # outputs.append(lines[-3])

        return outputs

def merge_lines_pipeline_2(lines):
    super_lines_final = []
    super_lines = []
    min_distance_to_merge = 30
    min_angle_to_merge = 30

    for line in lines:
        create_new_group = True
        group_updated = False

        for group in super_lines:
            for line2 in group:
                if get_distance(line2, line) < min_distance_to_merge:
                    # check the angle between lines
                    orientation_i = math.atan2((line[0][1]-line[1][1]),(line[0][0]-line[1][0]))
                    orientation_j = math.atan2((line2[0][1]-line2[1][1]),(line2[0][0]-line2[1][0]))

                    if int(abs(abs(math.degrees(orientation_i)) - abs(math.degrees(orientation_j)))) < min_angle_to_merge:
                        #print("angles", orientation_i, orientation_j)
                        #print(int(abs(orientation_i - orientation_j)))
                        group.append(line)

                        create_new_group = False
                        group_updated = True
                        break

            if group_updated:
                break

        if (create_new_group):
            new_group = []
            new_group.append(line)

            for idx, line2 in enumerate(lines):
                # check the distance between lines
                if get_distance(line2, line) < min_distance_to_merge:
                    # check the angle between lines
                    orientation_i = math.atan2((line[0][1]-line[1][1]),(line[0][0]-line[1][0]))
                    orientation_j = math.atan2((line2[0][1]-line2[1][1]),(line2[0][0]-line2[1][0]))

                    if int(abs(abs(math.degrees(orientation_i)) - abs(math.degrees(orientation_j)))) < min_angle_to_merge:
                        #print("angles", orientation_i, orientation_j)
                        #print(int(abs(orientation_i - orientation_j)))

                        new_group.append(line2)

                        # remove line from lines list
                        #lines[idx] = False
            # append new group
            super_lines.append(new_group)


    for group in super_lines:
        super_lines_final.append(merge_lines_segments1(group))

    return super_lines_final

def merge_lines_segments1(lines, use_log=False):
    if(len(lines) == 1):
        return lines[0]

    line_i = lines[0]

    # orientation
    orientation_i = math.atan2((line_i[0][1]-line_i[1][1]),(line_i[0][0]-line_i[1][0]))

    points = []
    for line in lines:
        points.append(line[0])
        points.append(line[1])

    if (abs(math.degrees(orientation_i)) > 45) and abs(math.degrees(orientation_i)) < (90+45):

        #sort by y
        points = sorted(points, key=lambda point: point[1])

        if use_log:
            print("use y")
    else:

        #sort by x
        points = sorted(points, key=lambda point: point[0])

        if use_log:
            print("use x")

    return [points[0], points[len(points)-1]]

# https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.distance.cdist.html
# https://stackoverflow.com/questions/32702075/what-would-be-the-fastest-way-to-find-the-maximum-of-all-possible-distances-betw
def lines_close(line1, line2):
    dist1 = math.hypot(line1[0][0] - line2[0][0], line1[0][0] - line2[0][1])
    dist2 = math.hypot(line1[0][2] - line2[0][0], line1[0][3] - line2[0][1])
    dist3 = math.hypot(line1[0][0] - line2[0][2], line1[0][0] - line2[0][3])
    dist4 = math.hypot(line1[0][2] - line2[0][2], line1[0][3] - line2[0][3])

    if (min(dist1,dist2,dist3,dist4) < 100):
        return True
    else:
        return False

def lineMagnitude (x1, y1, x2, y2):
    lineMagnitude = math.sqrt(math.pow((x2 - x1), 2)+ math.pow((y2 - y1), 2))
    return lineMagnitude

#Calc minimum distance from a point and a line segment (i.e. consecutive vertices in a polyline).
# https://nodedangles.wordpress.com/2010/05/16/measuring-distance-from-a-point-to-a-line-segment/
# http://paulbourke.net/geometry/pointlineplane/
def DistancePointLine(px, py, x1, y1, x2, y2):
    #http://local.wasp.uwa.edu.au/~pbourke/geometry/pointline/source.vba
    LineMag = lineMagnitude(x1, y1, x2, y2)

    if LineMag < 0.00000001:
        DistancePointLine = 9999
        return DistancePointLine

    u1 = (((px - x1) * (x2 - x1)) + ((py - y1) * (y2 - y1)))
    u = u1 / (LineMag * LineMag)

    if (u < 0.00001) or (u > 1):
        #// closest point does not fall within the line segment, take the shorter distance
        #// to an endpoint
        ix = lineMagnitude(px, py, x1, y1)
        iy = lineMagnitude(px, py, x2, y2)
        if ix > iy:
            DistancePointLine = iy
        else:
            DistancePointLine = ix
    else:
        # Intersecting point is on the line, use the formula
        ix = x1 + u * (x2 - x1)
        iy = y1 + u * (y2 - y1)
        DistancePointLine = lineMagnitude(px, py, ix, iy)

    return DistancePointLine

def get_distance(line1, line2):
    dist1 = DistancePointLine(line1[0][0], line1[0][1],
                            line2[0][0], line2[0][1], line2[1][0], line2[1][1])
    dist2 = DistancePointLine(line1[1][0], line1[1][1],
                            line2[0][0], line2[0][1], line2[1][0], line2[1][1])
    dist3 = DistancePointLine(line2[0][0], line2[0][1],
                            line1[0][0], line1[0][1], line1[1][0], line1[1][1])
    dist4 = DistancePointLine(line2[1][0], line2[1][1],
                            line1[0][0], line1[0][1], line1[1][0], line1[1][1])


    return min(dist1,dist2,dist3,dist4)




BlurType = Enum('BlurType', 'Box_Blur Gaussian_Blur Median_Filter Bilateral_Filter')
