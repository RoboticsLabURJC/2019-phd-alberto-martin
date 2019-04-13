from setuptools import setup, find_packages
import sys, os.path


def parse_requirements(filename):
    """ load requirements from a pip requirements file """
    lineiter = (line.strip() for line in open(filename))
    return [line for line in lineiter if line and not line.startswith("#")]


def copy_dir():
    dir_path = 'envs/gazebo/assets/models/'
    base_dir = os.path.join('gym_pyxis', dir_path)
    for (dirpath, dirnames, files) in os.walk(base_dir):
        for f in files:
            yield os.path.join(dirpath.split('/', 1)[1], f)

install_reqs = parse_requirements('requirements.txt')

reqs = [ir for ir in install_reqs]


# Don't import gym module here, since deps may not be installed
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'gym-pyxis'))

setup(name='gym-pyxis',
      version='0.1',
      install_requires=reqs,
      description='A package for custom gym environments',
      url='https://github.com/RoboticsURJC-students/2019-phd-alberto-martin',
      author='almartinflorido',
      packages = find_packages(exclude=['test']),
      package_data={'gym_pyxis':
                        ['envs/gazebo/assets/launch/*.launch',
                         'envs/gazebo/assets/worlds/*'],
                    '':[f for f in copy_dir()]},
      include_package_data=True,
      )