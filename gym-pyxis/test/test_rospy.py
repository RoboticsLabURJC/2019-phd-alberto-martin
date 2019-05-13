import os
import cv2
import rospy
import rosgraph
import socket
import logging
import unittest
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError

DEFAULT_NODE_PORT = 0  # bind to any open port
DEFAULT_MASTER_PORT = 11311  # default port for master's to bind to
DEFAULT_MASTER_URI = 'http://localhost:%s/' % DEFAULT_MASTER_PORT

logger = logging.getLogger(__name__)


class TestRospy(unittest.TestCase):

    def test_rospy(self):
        if "ROS_MASTER_URI" not in os.environ:
            os.environ["ROS_MASTER_URI"] = DEFAULT_MASTER_URI
            print('Ros master uri not defined')

        try:
            rosgraph.Master('/rostopic').getPid()
            print('Master comunication ok')
        except socket.error:
            self.assertEqual(True, False, "Unable to communicate with master!")

        vel_pub_service = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        rospy.init_node('test_rospy', anonymous=True)
        timeout = 5

        image_data = None
        cv_image = None
        while image_data is None:
            try:
                image_data = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=timeout)
                cv_image = CvBridge().imgmsg_to_cv2(image_data, "bgr8")
            except Exception as e:
                print('exception raised: {}'.format(e))
                logger.warning("TurtlebotEnv: exception raised getting camera data {}".format(e))

        cv2.imwrite('test.png', cv_image)

        # data = None
        # while data is None:
        #     try:
        #         data = rospy.wait_for_message('/scan', LaserScan, timeout=timeout)
        #     except Exception as e:
        #         print('exception raised: {}'.format(e))
        #         logger.warning("TurtlebotEnv: exception raised getting laser data {}".format(e))
        #
        # print(data)

        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        vel_pub_service.publish(vel_cmd)


if __name__ == '__main__':
    TestRospy().test_rospy()