# Sawyer Control
## Setup Instructions:
1. Make sure ros indigo is installed and make sure to add source /opt/ros/indigo/setup.bash to bashrc
2. Install intera interface from the rethink website
3. Make sure to switch to the 5.1.0 release branch before running catkin_make 
4. run ./intera.sh and then try to import intera_interface from system python
5. Git clone the following:
* Urdfdom: https://github.com/ros/urdfdom.git
* urdf_parser_py: https://github.com/ros/urdf_parser_py
* pykdl utils: https://github.com/gt-ros-pkg/hrl-kdl/tree/indigo-devel
6. run catkin_make
7. install gym 0.9.3 in your python3 env
8. Git clone rllab
9. Download mujoco, put the zip in rllab, run the setup mujoco script, put it in ~/.mujoco and unzip it there 

Example Bashrc:
```
source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash
export PYTHONPATH=~/Documents/rllab/:
export PATH=$PATH:/home/<user>/anaconda2/bin
export PYTHONPATH=$PYTHONPATH:/opt/ros/indigo/lib/python2.7/dist-packages/:
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
```

## Usage:

Basic workflow:

1. Make sure robot is enabled

2. open a new terminal and run the following commands:
``` 
saw
exp_nodes 
``` 

3. Now open another terminal and run your algorithm on the sawyer

In terms of running algorithms on the sawyer, you can simply plug in the sawyer environments directly into your launch script
the same you use any other gym environment: The sawyer env follows the same API. 

## Features to come:
* End Effector reaching environment
* Lego Block stacking environment
* End Effector position control 
* Vision based environments
