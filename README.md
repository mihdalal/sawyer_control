# Sawyer Control
## Setup Instructions:
1. Make sure ros kinetic is installed and make sure to add source /opt/ros/kinetic/setup.bash to your bashrc
2. Install intera interface from the rethink website and set up intera.sh with the correct ip and hostname of your robot
3. run `./intera.sh` and then try to import `intera_interface` from system python
4. Git clone the following in ~/catkin_ws/src/:
* Urdfdom: https://github.com/ros/urdfdom.git
* urdf_parser_py: https://github.com/ros/urdf_parser_py
* pykdl utils: https://github.com/gt-ros-pkg/hrl-kdl/tree/

5, switch to the indigo-devel branch on all three repos

6.`git clone sawyer_control: https://github.com/mdalal2020/sawyer_control.git` in ~/catkin_ws/src/

7. run `catkin_make`

8. Make sure you are on system python

9. run `pip install -r system_python_requirements.txt`

10. install anaconda 2 (Do not install anaconda 3!) and type no when it asks you prepend the anaconda path to the bashrc

11. manually add in the anaconda path to the bashrc (see example bashrc below)

12. run `conda create -n <env_name> python=3.5 anaconda`

13. source activate your python 3 conda environment

14. run `pip install -r python3_requirements.txt`

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

## Usage:

Basic workflow:

1. Make sure robot is enabled

2. open a new terminal and run the following commands:
``` 
saw
exp_nodes 
``` 

3. Now open another terminal/tab and run your algorithm on the sawyer

In terms of running algorithms on the sawyer, you can simply plug in the sawyer environments directly into your launch script
the same you use any other gym environment: The sawyer env follows the OpenAI Gym API. 

## Features:
* Torque and Position Control Modes
* End Effector Position Reaching Environment
* Vision Wrapper 
* Gym-Style Interface 
