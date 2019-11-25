Clone this repository in your ~/catkin_ws/src folder

(Rename the directory to group_3)

Get the permissions:
chmod u+x ~/catkin_ws/src/group_3/scripts/*.py
chmod u+x ~/catkin_ws/src/group_3/env_setup.sh && ~/catkin_ws/src/group_3/env_setup.sh

To move the bot according to policy:
1. Run `roscore`
2. Run `rosrun group_3 server.py -d 5 -s 6`
    where,
            -d : number of dirty cells
            -s : size of the grid
3. Run `roslaunch group_3 maze.launch`
4. Run `rosrun group_3 move_tbot3.py`
5. Run `rosrun group_3 Project_code.py`


To start the demo, run the following commands in order:
1. Run `roscore`
2. Run `rosrun group_3 server.py -d 5 -s 6`
    where,
            -d : number of dirty cells
            -s : size of the grid
3. Run `roslaunch group_3 maze.launch`
4. Run `rosrun group_3 move_tbot3.py`
5. Run `rosrun group_3 random_walk.py`
