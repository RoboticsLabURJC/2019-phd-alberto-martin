import cv2
from gym_pyxis.envs.gazebo.f1 import F1


if __name__ == '__main__':
    f1 = F1()
    while True:

        img = f1.get_camera_data()
        cv2.imshow('image', img)
        cv2.waitKey(1)

