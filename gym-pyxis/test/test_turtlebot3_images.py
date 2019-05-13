import cv2
import unittest
from gym_pyxis.envs.gazebo.turtlebot import turtlebot_utils as utils


class TestTurtlebot3Images(unittest.TestCase):

    def test_turtlebot3_images(self):

        img = cv2.imread('image.png')
        robot_position = utils.get_robot_position_respect_road(img, debug=True)

        if robot_position == 'center_road':
            reward = 2
        elif robot_position == 'in_road':
            reward = 0.5
        else:
            reward = -1

        print('reward: {}'.format(reward))