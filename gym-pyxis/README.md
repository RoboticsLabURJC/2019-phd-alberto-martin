# Open Ai Gym extension for robotics
Inspired by [gym_gazebo](https://github.com/erlerobot/gym-gazebo), gym_pyxis is a customized gym extension that includes environments for gazebo and stage simulator.
## Installation
### Requirements
+ [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (python3)
+ sudo apt install python-catkin-tools
+ copy models to gazebo model path
+ Player (3.02) and Stage (4.1.1) with python support. 
+ OpenAI Gym


[Turtlebot3 simulation](http://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#turtlebot3-simulation-using-fake-node)

## Player/Stage installation
[How to use Player/Stage](http://player-stage-manual.readthedocs.io/en/latest/)
### Player
```bash
$ git clone https://github.com/playerproject/player
$ cd player
$ mkdir build
$ cd build
$ cmake -jn -DBUILD_PYTHONCPP_BINDINGS=ON ..
$ sudo make install -jn
```
### Stage
```bash
$ git clone https://github.com/rtv/Stage
$ cd Stage
$ mkdir build
$ cd build
$ cmake -jn ..
$ sudo make install -jn
```
## Execution
```bash
$ export LD_LIBRARY_PATH=/usr/local/lib64/:$LD_LIBRARY_PATH
$ cd /usr/local/share/stage/worlds
$ player simple.cfg
```

## FAQ

- **Player**: Compilation error on Ubuntu 18.04. The following library is missing:

  ```bash
  /usr/bin/ld: cannot find -lgeos
  ```
  To solve this, install the following library:
  
  ```bash
  sudo apt install libgeos++-dev
  ```

  More information about the error in [this link](https://github.com/playerproject/player/issues/18).

- **Stage**: Compilation error on Ubuntu 18.04. The following library is missing:

  ```bash
  Could NOT find FLTK (missing: FLTK_LIBRARIES)
  ```

  To solve this, install the following depencences:

  ```bash
  sudo apt install fltk1.3-dev
  sudo apt install libxft-dev
  sudo apt install libxinerama-dev
  ```

  More information about the error in [this link](https://stackoverflow.com/questions/25752961/how-to-compile-with-fltk-under-ubuntu).

- **Error in execution (Player)**. The terminal returns the following error:

  ```bash
  Player v.3.1.1-dev
  
  * Part of the Player/Stage/Gazebo Project [http://playerstage.sourceforge.net].
  * Copyright (C) 2000 - 2013 Brian Gerkey, Richard Vaughan, Andrew Howard,
  * Nate Koenig, and contributors. Released under the GNU General Public License.
  * Player comes with ABSOLUTELY NO WARRANTY.  This is free software, and you
  * are welcome to redistribute it under certain conditions; see COPYING
  * for details.
  
  error   : Failed to load plugin stageplugin.
  error   : libtool reports error: file not found
  error   : plugin search path: /usr/local/share/stage/worlds:/usr/local/share/stage/worlds:/usr/local/lib/player-3.1/:/usr/local/lib/
  error   : failed to load plugin: stageplugin
  error   : failed to parse config file /usr/local/share/stage/worlds/simple.cfg driver blocks
  ```

  To solve this, change the environment variable to this one:

  ```
  export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH
  ```

  The library is in `/usr/local/lib/` instead of `/usr/local/lib64/`.