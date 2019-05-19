import cv2
import unittest
from gym_pyxis.envs.gazebo.f1 import F1


class TestF1(unittest.TestCase):

    def test_f1_model(self):

        f1 = F1()
        f1.send_velocity_command(0.0, 0.0)
        f1.reset_simulation()
        # f1.get_odometry()
        image = f1.get_camera_data()
        print(image.shape)
        f1.set_pose(100.0, 0, 0, 0, 0, 0, 0, model_name='f1_renault')

        f1.send_velocity_command(1.0, 0.0)
        #
        #
        #
        # f1.get_odometry()
        f1.send_velocity_command(0.0, 0.0)
        # cv2.imshow('f1_camera', image)
        # cv2.waitKey(0)

