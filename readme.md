**Brief Description of the project**

We have created a Grid world environment where a few cells contain dirt (represented by patches in the cell).
It is a Fully observable environment which means the initial configuration of the grid including the dirt locations is known
Actions are Stochastic and it's a Single Agent program.
The objective of the agent is to clean the entire grid in an optimal way

**Instructions to execute this project**

Save this folder in your ~/catkin_ws/src folder

Get the permissions:
chmod u+x ~/catkin_ws/src/group_3/scripts/*.py
chmod u+x ~/catkin_ws/src/group_3/env_setup.sh && ~/catkin_ws/src/group_3/env_setup.sh

1. Run `roscore`
2. Run `rosrun group_3 server.py -d 5 -s 6`
    where,
            -d : number of dirty cells
            -s : size of the grid
Note: d should not be more than the number of dirt locations 
3. Run `roslaunch group_3 maze.launch`
4. Run `rosrun group_3 move_tbot3.py`
5. Run `rosrun group_3 Project_code.py`


To try the random_walk execution, run the following commands in order:
1. Run `roscore`
2. Run `rosrun group_3 server.py -d 5 -s 6`
    where,
            -d : number of dirty cells
            -s : size of the grid
3. Run `roslaunch group_3 maze.launch`
4. Run `rosrun group_3 move_tbot3.py`
5. Run `rosrun group_3 random_walk.py`
