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


def draw_regions(roi, left_x_safe_region, right_x_safe_region, left_x_unsafe_region, right_x_unsafe_region):
    img_height, img_width, _ = roi.shape

    cv2.line(roi, (left_x_safe_region, 0), (left_x_safe_region, img_height), (0,255,0), 2)
    cv2.line(roi, (right_x_safe_region, 0), (right_x_safe_region, img_height), (0, 255, 0), 2)

    cv2.line(roi, (left_x_unsafe_region, 0), (left_x_unsafe_region, img_height), (0,0,255), 2)
    cv2.line(roi, (right_x_unsafe_region, 0), (right_x_unsafe_region, img_height), (0, 0, 255), 2)


def get_image_region(img, debug=False):
    img_height, img_widht, _ = img.shape

    roi_y_max = img_height - 20
    roi_y_min = roi_y_max - 100

    input_image_grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    roi = img[roi_y_min:roi_y_max, :, :]
    roi_grayscale = input_image_grayscale[roi_y_min:roi_y_max, :]

    ret, th1 = cv2.threshold(roi_grayscale, 127, 255, cv2.THRESH_BINARY)

    roi_height, roi_width = th1.shape
    middle_x = roi_width / 2.0
    padding_px = 10

    left_x_safe_region = middle_x - padding_px
    right_x_safe_region = middle_x + padding_px
    left_x_unsafe_region = left_x_safe_region - (padding_px * 4)
    right_x_unsafe_region = right_x_safe_region + (padding_px * 4)

    if debug:
        cv2.imshow('input', img)
        cv2.imshow('th', th1)
        draw_regions(roi, int(left_x_safe_region), int(right_x_safe_region), int(left_x_unsafe_region),
                     int(right_x_unsafe_region))
        cv2.imshow('regions', roi)

    bin, contours, hierarchy = cv2.findContours(th1, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    rects = {}
    for c in contours:
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02 * peri, True)
        x, y, w, h = cv2.boundingRect(approx)
        rect = (x, y, w, h)
        rects[w*h] = rect
        if debug:
            cv2.rectangle(roi, (x,y), (x+w, y+h), (0,255,255), 1)

    if len(rects) > 0:
        biggest_rect = rects[list(rects.keys())[-1]]
        middle_rect_x = biggest_rect[0] + (biggest_rect[2] / 2.0)

        if debug:
            cv2.line(roi, (int(middle_rect_x), 0), (int(middle_rect_x), roi_height), (255, 0, 255), 2)
            cv2.imshow('roi', roi)
            cv2.waitKey(0)

        if middle_rect_x > left_x_safe_region and middle_rect_x < right_x_safe_region:
            return 'safe_region'
        elif middle_rect_x > left_x_unsafe_region and middle_rect_x < left_x_safe_region or \
             middle_rect_x < right_x_unsafe_region and middle_rect_x > right_x_safe_region:
             return 'unsafe_region'
        else:
            return 'out_road'
    else:
        return 'out_road'
