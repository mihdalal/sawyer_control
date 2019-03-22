# Sawyer Control
## Authors:
Murtaza Dalal and Shikhar Bahl 
## Description
Sawyer Control is a repository that enables RL algorithms to control Rethink Sawyer robots via an OpenAI Gym Style interface. It is both a ROS package and a set of Sawyer (Gym) Envs combined in one. The ROS portion of the repo handles the actual control of the robot and is executed in Python 2. The environments are all in Python 3, and communicate to the robot via the ROS interface. Currently, this repo is capable of utilizing both the state information of the robot as well as visual input from a Microsoft Kinect sensor. 

## Setup Instructions:
1. Make sure ros kinetic is installed and make sure to add source /opt/ros/kinetic/setup.bash to your bashrc
2. Install intera interface from the rethink website and set up intera.sh with the correct ip and hostname of your robot
3. Git clone the following in ~/catkin_ws/src/:
* Urdfdom: https://github.com/ros/urdfdom.git
* urdf_parser_py: https://github.com/ros/urdf_parser_py
* pykdl utils: https://github.com/gt-ros-pkg/hrl-kdl
4. switch to the indigo-devel branch on urdf_parser_py and hrl_kdl
5. run `git clone https://github.com/mdalal2020/sawyer_control.git` in ~/catkin_ws/src/
6. run `catkin_make`
7. Make sure you are on system python
8. run `pip install -r system_python_requirements.txt`
9. install anaconda 2 (Do not install anaconda 3!) and type no when it asks you prepend the anaconda path to the bashrc
10. manually add in the anaconda path to the bashrc (see example bashrc below)
11. run `conda create -n <env_name> python=3.5 anaconda`
12. source activate your python 3 conda environment
13. run `pip install -r python3_requirements.txt`
14. install kinect2 bridge: https://github.com/code-iai/iai_kinect2/tree/master/kinect2_bridge

Example Bashrc:
```
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
export PATH="$PATH:$HOME/anaconda2/bin"
export PYTHONPATH=$PYTHONPATH:/opt/ros/kinetic/lib/python2.7/dist-packages/:
```
Useful aliases:
```
alias saw="cd ~/catkin_ws/; ./intera.sh; cd ~/"
alias enable="rosrun intera_interface enable_robot.py -e"
alias disable="rosrun intera_interface enable_robot.py -d"
alias reset="rosrun intera_interface enable_robot.py -r"
alias stop="rosrun intera_interface enable_robot.py -S"
alias status="rosrun intera_interface enable_robot.py -s"
alias exp_nodes="roslaunch ~/catkin_ws/src/sawyer_control/exp_nodes.launch"
alias kinect="roslaunch kinect2_bridge kinect2_bridge.launch"
```

## Environments
All environments inherit from `SawyerEnvBase` which contains all of the core functionality that is central to using the Sawyer Robot including different control modes, robot state observations, and safety boxes. Note, unless you have a good reason not to, you should ALWAYS have the safety box enabled, ie set `use_safety_box=True`. This environment also provides functionality for changing various settings (see configs section) as well as having a fixed goal (as opposed to the default functionality, which is multi-goal). 

There are two main control modes for the Sawyer robot, torque and position, each of which has differing functionality and settings. In terms of the actual controller, all of the actions are executed using intera's in built torque/joint position controller, and the environment merely provides an abstraction around this. For the torque control mode, changing around the `torque_action_scale` will be quite important (generally larger values around 5 or so are better) and the control frequency is 20Hz. The position control mode in the environment does end-effector position control, an option that intera does not provide. As a result, given a desired end-effector position, the environment uses inverse kinematics to compute the corresponding joint angles, and commands the intera impedance controller to move to those angles. The position controller can go as fast you like (by setting the `max_speed` option in `SawyerEnvBase`). Setting higher max speeds will result in a loss of accuracy, so I would recommend testing each speed on a variety of different reaching tasks to see what level of accuracy works for you. In general, I've found anything between .1-.4 works pretty well if your action magnitudes are up to 1cm. You can set this option using `position_action_scale` in `SawyerEnvBase`. 

Currently the repository holds two main environments, `SawyerReachXYZEnv` and `SawyerPushXYEnv`. For `SawyerReachXYZEnv` the goal is to reach a target end-effector position. For `SawyerPushXYEnv` the goal is to push an object to a desired position. Since the environment is in the real world, for which we don't have access to the state information of the object to push, you must wrap this environment with `ImageEnv`. See the usage section for how to to this.

## Usage:

### Basic workflow:

1. Make sure robot is enabled

2. open a new terminal and run the following commands:
``` 
saw
exp_nodes 
``` 
3. (Optional) open another terminal/tab and run the following commands:
```
saw 
kinect
```
3. Now open another terminal/tab and run your algorithm on the sawyer. Note you must run `saw` in any new tab that accesses environments/scripts in `sawyer_control`. 

In terms of running algorithms on the sawyer, you can simply plug in the sawyer environments directly into your launch script in the same way you use any other gym environment: The sawyer environments follow the OpenAI Gym API. Additionally, these environments are Multitask by default, which means that they resample goals on reset and have goal conditioned rewards. If you just want a single goal, set `fix_goal=True` and provide the goal you desire to the environment in `fixed_goal`.  

### Using Environments

To import the reaching or pushing environment:

```
from sawyer_control.envs.sawyer_reaching import SawyerReachXYZEnv
env = SawyerReachXYZEnv(...)
env.step(action=...)
```

To use the `ImageEnv` wrapper (which is compatible with any SawyerEnv):
```
from sawyer_control.envs.sawyer_pushing import SawyerPushXYEnv
from sawyer_control.core import ImageEnv
env = SawyerPushXYEnv(...)
image_env = ImageEnv(env, ...)
image = env.get_flat_image()
```

### Configs:
All the important/hardcoded settings for robot/env details are stored in the config files. Please do not change the ros_config or base_config files. If you wish to modify settings, make a new configuration file and have it import all the standard configs and modify the rest, see `austri_config.py` for an example. Then add the file to the config dictionary in `config.py` and simply pass in the name to the env to obtain the desired settings. 

### Common Problems:
Q: The robot doesn't move!

A: Double check that you ran `exp_nodes` before running the experiment

Q: I ran `exp_nodes` and the robot still doesn't move!

A: Run `status` and check if `Ready=False`. If so the robot is in homing mode (Try pressing the grey button on the robot and moving the arm around, alternatively you might just have to re-start until this error goes away. Unfortunately, I don't have a consistent fix for this). 

Q: The arm is just moving upwards all the time in torque control mode. 

A: You probably need to up the `torque_action_scale`, I recommend something like 4/5, but it depends on how safe you want the environment and how constrained your space is. The problem occurs because you are applying too small torques, so the solution is to apply larger torques. 


## Features:
* Torque and Position Control Modes
* End-Effector Reaching Environment
* Pushing Environment
* Vision Wrapper 
* Gym-Style Interface 
