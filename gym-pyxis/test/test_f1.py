import cv2
import unittest
from gym_pyxis.envs.gazebo.f1 import F1


class TestTurtlebot3Images(unittest.TestCase):

    def test_turtlebot3_images(self):

        f1 = F1()
        f1.send_velocity_command(0.0, 0.1)



        image = f1.get_camera_data()
        print(image.shape)
        # cv2.imshow('f1_camera', image)
        # cv2.waitKey(0)

