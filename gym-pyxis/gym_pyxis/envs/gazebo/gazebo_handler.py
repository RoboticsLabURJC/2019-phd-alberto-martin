import os
import rospy
import rosgraph
import logging
import socket
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

logger = logging.getLogger(__name__)


class GazeboHandler:

    DEFAULT_NODE_PORT = 0  # bind to any open port
    DEFAULT_MASTER_PORT = 11311  # default port for master's to bind to
    DEFAULT_MASTER_URI = 'http://localhost:%s/' % DEFAULT_MASTER_PORT

    def __init__(self, node_name):
        self.node_name = node_name

        if "ROS_MASTER_URI" not in os.environ:
            os.environ["ROS_MASTER_URI"] = GazeboHandler.DEFAULT_MASTER_URI
            logger.warning('ROS master URI not defined. Setting {} as default ROS master'.format(
                GazeboHandler.DEFAULT_MASTER_URI))

        try:
            rosgraph.Master('/rostopic').getPid()
            logger.info('ROS master communication: ok')
        except socket.error as e:
            logging.error("Unable to communicate with ROS master!")
            raise e

        try:
            rospy.init_node(node_name, anonymous=True)
            logger.info("{}: gym ROS node created.".format(node_name))
        except rospy.exceptions.ROSException as re:
            logger.error("{}: exception raised creating gym ROS node. {}".format(node_name, re))
            raise re

        self.un_pause_service = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause_service = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_service = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        self.set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def reset_simulation(self):
        rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            self.reset_service()
        except rospy.ServiceException as e:
            logger.warning("{}: exception raised reset simulation {}".format(self.node_name, e))

    def pause_physics(self):
        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause_service()
        except rospy.ServiceException as e:
            logger.warning("{}: exception raised pause physics {}".format(self.node_name, e))

    def un_pause_physics(self):
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            self.un_pause_service()
        except rospy.ServiceException as e:
            logger.warning("{}: exception raised unpause physics {}".format(self.node_name, e))

    def set_pose(self, x, y, z, q0, q1, q2, q3, model_name):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        pose.orientation.x = q0
        pose.orientation.y = q1
        pose.orientation.z = q2
        pose.orientation.w = q3

        state = ModelState()
        state.model_name = model_name
        state.pose = pose

        try:
            self.set_model_state(state)
        except Exception as e:
            logger.warning("{}: exception raised setting robot pose {}".format(self.node_name, e))
