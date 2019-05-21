import cv2
import numpy as np


def draw_region_and_road_center(roi, left_x_safe_region, right_x_safe_region, road_x, road_y):
    img_height, img_width, _ = roi.shape

    cv2.line(roi, (left_x_safe_region, 0), (left_x_safe_region, img_height), (0, 255, 0), 2)
    cv2.line(roi, (right_x_safe_region, 0), (right_x_safe_region, img_height), (0, 255, 0), 2)
    if road_y >= 0 and road_x >= 0:
        cv2.circle(roi, (road_x, road_y), 10, (0, 0, 255), 2)


def get_robot_position_respect_road(img, debug=False):
    img_height, img_width, _ = img.shape

    roi_y_max = img_height - 20
    roi_y_min = roi_y_max - 100

    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])

    input_image_hsv = cv2.cvtColor(img.copy(), cv2.COLOR_BGR2HSV)
    roi = img[roi_y_min:roi_y_max, :, :]
    roi_hsv = input_image_hsv[roi_y_min:roi_y_max, :]
    th1 = cv2.inRange(roi_hsv, lower_red, upper_red)

    road_detected = False
    cX = -1
    cY = -1
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

    robot_position = 'out_road'
    if in_center_of_road:
        robot_position = 'center_road'
    elif not out_road:
        robot_position = 'in_road'

    if debug:
        cv2.putText(img, robot_position, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)
        cv2.imshow('input', img)
        cv2.imshow('th', th1)
        draw_region_and_road_center(roi, int(line_center_left), int(line_center_right), cX, cY)
        cv2.imshow('regions', roi)
        # cv2.waitKey(0)

    return robot_position