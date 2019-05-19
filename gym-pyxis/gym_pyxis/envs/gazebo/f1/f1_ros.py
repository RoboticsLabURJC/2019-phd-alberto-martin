import rospy
import logging
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from gym_pyxis.envs.gazebo import GazeboHandler

logger = logging.getLogger(__name__)


class F1(GazeboHandler):

    def __init__(self):
        node_name = 'f1_ros'
        super(F1, self).__init__(node_name)
        self.vel_pub_service = rospy.Publisher('/F1ROS/cmd_vel', Twist, queue_size=5)

    @staticmethod
    def get_camera_data(timeout=5, max_retries=5):
        # (480, 640, 3)
        image_data = None
        cv_image = None
        retry_count = 0
        while image_data is None:
            try:
                image_data = rospy.wait_for_message('/F1ROS/cameraL/image_raw', Image, timeout=timeout)
                cv_image = CvBridge().imgmsg_to_cv2(image_data, "bgr8")
            except Exception as e:
                logger.warning("f1_ros: exception raised getting camera data {}".format(e))
                retry_count += 1
                if retry_count >= max_retries:
                    raise e

        return cv_image

    @staticmethod
    def get_odometry(timeout=5, max_retries=5):
        data = None
        retry_count = 0
        while data is None:
            try:
                data = rospy.wait_for_message('/F1ROS/odom', Odometry, timeout=timeout)
            except Exception as e:
                logger.warning("f1_ros: exception raised getting odometry data {}".format(e))
                retry_count += 1
                if retry_count >= max_retries:
                    raise e

        return data

    def send_velocity_command(self, linear_x, angular_z):
        vel_cmd = Twist()
        vel_cmd.linear.x = linear_x
        vel_cmd.angular.z = angular_z
        self.vel_pub_service.publish(vel_cmd)
