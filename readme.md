Clone this repository in your ~/catkin_ws/src folder

Rename the directory to cse571_project

chmod u+x ~/catkin_ws/src/cse571_project/scripts/*.py

chmod u+x ~/catkin_ws/src/cse571_project/env_setup.sh && ~/catkin_ws/src/cse571_project/env_setup.sh

To test:
1. Run `roscore`
2. Run `rosrun cse571_project server.py -d 5 -s 6`
3. Run `roslaunch cse571_project maze.launch`
4. Run `rosrun cse571_project move_tbot3.py`
5. Run `rosrun cse571_project Project_code.py`


To start the demo, run the following commands in order:
1. Run `roscore`
2. Run `rosrun cse571_project server.py -d 5 -s 6`
3. Run `roslaunch cse571_project maze.launch`
4. Run `rosrun cse571_project move_tbot3.py`
5. Run `rosrun cse571_project random_walk.py`
