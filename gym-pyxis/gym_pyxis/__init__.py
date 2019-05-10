import os
import logging
from gym.envs.registration import register
logger = logging.getLogger(__name__)

# Check ROS envrionment
if "ROS_MASTER_URI" not in os.environ:
    logger.warning("ROS_MASTER_URI not found. Have you initialize ROS environment?")

# Gazebo
# ----------------------------------------

# Turtlebot envs
register(
    id='Turtlebot3FollowLineCameraEnv-v0',
    entry_point='gym_pyxis.envs.gazebo.turtlebot:Turtlebot3FollowLineCameraEnv',
    # More arguments here
)

register(
    id='WorldPlayerStageEnv-v0',
    entry_point='gym_pyxis.envs.player_stage:WorldPlayerStageEnv',
    # More arguments here
)

register(
    id='SimpleRoomsEnv-v0',
    entry_point='gym_pyxis.envs.grid_worlds:SimpleRoomsEnv',
    # More arguments here
)

register(
    id='CliffWalkingEnv-v0',
    entry_point='gym_pyxis.envs.grid_worlds:CliffWalkingEnv',
    # More arguments here
)

register(
    id='WindyGridworldEnv-v0',
    entry_point='gym_pyxis.envs.grid_worlds:WindyGridworldEnv',
    # More arguments here
)