from setuptools import setup, find_packages
from pip.req import parse_requirements

import sys, os.path
install_reqs = parse_requirements('requirements.txt')
reqs = [str(ir.req) for ir in install_reqs]


# Don't import gym module here, since deps may not be installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'gym-pyxis'))

setup(name='gym-pyxis',
      version='0.1',
      install_requires=reqs,
      description='A package for custom gym environments',
      url='https://github.com/RoboticsURJC-students/2019-phd-alberto-martin',
      author='almartinflorido',
      packages = find_packages(),
      package_data={'gym_pyxis': ['envs/gazebo/assets/launch/*.launch', 'envs/gazebo/assets/worlds/*']},
)