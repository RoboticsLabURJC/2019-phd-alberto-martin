import cv2
import numpy as np

def discretize_observation(data, new_ranges, min_range = 0.2):
    discretized_ranges = []
    done = False
    mod = len(data.ranges) / new_ranges
    for i, item in enumerate(data.ranges):
        if (i % mod == 0):
            if data.ranges[i] == float('Inf') or np.isinf(data.ranges[i]):
                discretized_ranges.append(6)
            elif np.isnan(data.ranges[i]):
                discretized_ranges.append(0)
            else:
                discretized_ranges.append(int(data.ranges[i]))
        if (min_range > data.ranges[i] > 0):
            done = True
    return discretized_ranges, done


def draw_region_and_road_center(roi, left_x_safe_region, right_x_safe_region, road_x, road_y):
    img_height, img_width, _ = roi.shape

    cv2.line(roi, (left_x_safe_region, 0), (left_x_safe_region, img_height), (0, 255, 0), 2)
    cv2.line(roi, (right_x_safe_region, 0), (right_x_safe_region, img_height), (0, 255, 0), 2)

    cv2.circle(roi, (road_x, road_y), 10, (0, 0, 255), 2)


def get_robot_position_respect_road(img, debug=False):
    img_height, img_width, _ = img.shape

    roi_y_max = img_height - 20
    roi_y_min = roi_y_max - 100

    input_image_grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    roi = img[roi_y_min:roi_y_max, :, :]
    roi_grayscale = input_image_grayscale[roi_y_min:roi_y_max, :]

    ret, th1 = cv2.threshold(roi_grayscale, 127, 255, cv2.THRESH_BINARY)

    road_detected = False
    if np.sum(th1) > 0:
        road_detected = True

        M = cv2.moments(th1)
        cX = int(M["m10"] / M["m00"])
        cY = int(M["m01"] / M["m00"])

    roi_height, roi_width = th1.shape
    middle_x = int(roi_width / 2.0)

    out_road = False
    if th1[roi_height - 1, middle_x] == 0:
        out_road = True

    padding_px = 40

    line_center_left = middle_x - padding_px
    line_center_right = middle_x + padding_px

    in_center_of_road = False
    if road_detected:
        if cX > line_center_left and cX < line_center_right and road_detected:
            in_center_of_road = True

    if debug:
        cv2.imshow('input', img)
        cv2.imshow('th', th1)
        draw_region_and_road_center(roi, int(line_center_left), int(line_center_right), cX, cY)
        cv2.imshow('regions', roi)
        cv2.waitKey(0)

    if in_center_of_road:
        return 'center_road'
    elif not out_road:
        return 'in_road'

    return 'out_road'


