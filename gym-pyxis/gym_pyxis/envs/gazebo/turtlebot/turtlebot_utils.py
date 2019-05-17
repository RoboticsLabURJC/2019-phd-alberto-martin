import cv2
import math
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
    if road_y >= 0 and road_x >= 0:
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
    # elif not out_road:
    #     robot_position = 'in_road'

    if debug:
        cv2.putText(img, robot_position, (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 2, (255,255,255), 2, cv2.LINE_AA)
        cv2.imshow('input', img)
        cv2.imshow('th', th1)
        draw_region_and_road_center(roi, int(line_center_left), int(line_center_right), cX, cY)
        cv2.imshow('regions', roi)
        # cv2.waitKey(0)

    return robot_position


def estimate_movement(frame, old_frame):
    cv2.imshow('frame', frame)
    cv2.imshow('old_frame', old_frame)

    feature_params = dict(maxCorners=100,
                          qualityLevel=0.3,
                          minDistance=7,
                          blockSize=7)

    lk_params = dict(winSize=(15, 15),
                     maxLevel=2,
                     criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    old_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    p0 = cv2.goodFeaturesToTrack(old_gray, mask=None, **feature_params)

    frame_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
    p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, frame_gray, p0, None, **lk_params)

    # Select good points
    good_new = p1[st==1]
    good_old = p0[st==1]

    # Create a mask image for drawing purposes
    mask = np.zeros_like(old_frame)
    color = np.random.randint(0, 255, (100, 3))
    directions = []
    x_values = []
    y_values = []
    for i,(new,old) in enumerate(zip(good_new, good_old)):
        a,b = new.ravel()
        c,d = old.ravel()
        v = np.array((c -a , b -d ))
        v = v / np.linalg.norm(v)
        directions.append(v)
        x_values.append(v[0])
        y_values.append(v[1])
        mask = cv2.line(mask, (a,b),(c,d), color[i].tolist(), 2)
        frame = cv2.circle(frame,(a,b),5,color[i].tolist(),-1)

    x_values.sort()
    y_values.sort()
    x, _ = np.histogram(x_values, bins=4)
    y, _ = np.histogram(y_values, bins=4)

    predominan_direction = np.array((x_values[np.amax(x)], y_values[np.amax(y)]))

    img = cv2.add(frame,mask)
    cv2.imshow('optical', img)
    # cv2.waitKey(0)

    a = 0