import rospy
import logging
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge

from gym_pyxis.envs.gazebo import GazeboHandler

logger = logging.getLogger(__name__)


class Turtlebot3(GazeboHandler):

    def __init__(self):
        self.node_name = 'turtlebot3'
        super(Turtlebot3, self).__init__(self.node_name)
        self.vel_pub_service = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    @staticmethod
    def get_laser_data(timeout=5):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=timeout)
            except Exception as e:
                logger.warning("turtlebot3: exception raised getting laser data {}".format(e))

        return data

    @staticmethod
    def get_camera_data(timeout=5):
        # (480, 640, 3)
        image_data = None
        cv_image = None
        while image_data is None:
            try:
                image_data = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=timeout)
                cv_image = CvBridge().imgmsg_to_cv2(image_data, "bgr8")
            except Exception as e:
                logger.warning("turtlebot3: exception raised getting camera data {}".format(e))

        return cv_image

    def send_velocity_command(self, linearx, angularz):
        vel_cmd = Twist()
        vel_cmd.linear.x = linearx
        vel_cmd.angular.z = angularz
        self.vel_pub_service.publish(vel_cmd)
