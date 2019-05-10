import os
import sys
import rospy
import logging
from std_srvs.srv import Empty
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError

logger = logging.getLogger(__name__)


class Turtlebot3:

    DEFAULT_NODE_PORT = 0  # bind to any open port
    DEFAULT_MASTER_PORT = 11311  # default port for master's to bind to
    DEFAULT_MASTER_URI = 'http://localhost:%s/' % DEFAULT_MASTER_PORT

    def __init__(self):
        try:
            rospy.init_node('turtlebot3', anonymous=True)
            logger.info("GazeboEnv: gym ROS node created.")
        except rospy.exceptions.ROSException as re:
            logger.error("GazeboEnv: exception raised creating gym ROS node. {}".format(re))
            self.close()
            sys.exit(-1)

        if 'TURTLEBOT3_MODEL' not in os.environ:
            os.environ['TURTLEBOT3_MODEL'] = 'waffle_pi'

        self.vel_pub_service = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        self.unpause_service = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_service = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_service = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

    def reset_simulation(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_service()
        except rospy.ServiceException as e:
            logger.warning("TurtlebotEnv: exception raised reset simulation {}".format(e))

    def pause_physics(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_service()
        except rospy.ServiceException as e:
            logger.warning("TurtlebotEnv: exception raised pause physics {}".format(e))

    def unpause_physics(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.unpause_service()
        except rospy.ServiceException as e:
            logger.warning("TurtlebotEnv: exception raised unpause physics {}".format(e))

    def get_laser_data(self, timeout=5):
        data = None
        while data is None:
            try:
                data = rospy.wait_for_message('/scan', LaserScan, timeout=timeout)
            except Exception as e:
                logger.warning("TurtlebotEnv: exception raised getting laser data {}".format(e))

        return data

    def get_camera_data(self, timeout=5):
        image_data = None
        cv_image = None
        while image_data is None:
            try:
                image_data = rospy.wait_for_message('/camera/rgb/image_raw', Image, timeout=timeout)
                cv_image = CvBridge().imgmsg_to_cv2(image_data, "bgr8")
            except Exception as e:
                logger.warning("TurtlebotEnv: exception raised getting camera data {}".format(e))

        return cv_image

    def send_velocity_command(self, linearx, angularz):
        vel_cmd = Twist()
        vel_cmd.linear.x = linearx
        vel_cmd.angular.z = angularz
        self.vel_pub_service.publish(vel_cmd)
