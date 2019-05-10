import cv2
import unittest
from gym_pyxis.envs.gazebo.turtlebot import turtlebot_utils as utils


class TestTurtlebot3Images(unittest.TestCase):

    def test_turtlebot3_images(self):

        img = cv2.imread('image.png')
        image_region = utils.get_image_region(img, True)

        if image_region == 'safe_region':
            reward = 2
        elif image_region == 'unsafe_region':
            reward = 0.5
        else:
            reward = 0

        print('reward: {}'.format(reward))