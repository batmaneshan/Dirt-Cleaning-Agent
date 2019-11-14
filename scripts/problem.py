#!/usr/bin/env python
# encoding: utf-8

__copyright__ = "Copyright 2019, AAIR Lab, ASU"
__authors__ = ["Naman Shah", "Chirav Dave", "Ketan Patil"]
__credits__ = ["Siddharth Srivastava"]
__license__ = "MIT"
__version__ = "1.0"
__maintainers__ = ["Pulkit Verma", "Abhyudaya Srinet"]
__contact__ = "aair.lab@asu.edu"
__docformat__ = 'reStructuredText'

import sys
import rospy
from dirt_cleaning_agent.srv import *
from std_msgs.msg import String

class State:
    """
    This class defines the state of the TurtleBot.

    """

    def __init__(self,x,y,orientation):
        """
        :param x: current x-cordinate of turtlebot
        :type x: float
        :param y: current x-cordinate of turtlebot
        :type y: float   
        :param orientation: current orientation of turtlebot, can be either NORTH, SOUTH, EAST, WEST
        :type orientation: float

        """  
        self.x  = x 
        self.y = y
        self.orientation = orientation

    def __eq__(self,other):
        if self.x == other.x and self.y == other.y and self.orientation == other.orientation:
            return True
        else:
            return False

    def __repr__(self):
        return "({}, {}, {})".format(str(self.x), str(self.y), str(self.orientation))

class Helper:
    """
    This class provides the methods used to control TurtleBot.
        
    Example:
        .. code-block:: python

            from problem import Helper

            h = Helper()
            init_state = h.get_init_state()

    """

    def execute_pick_action(self, book_name, state):
        """
        This action picks the book named book_name located at State robot_state.

        :param book_name: Name of the book
        :type book_name: str
        :param robot_state: State of the robot.
        :type robot_state:  State

        :returns: True if the place action was successful. False, otherwise. 
        :rtype: bool

        :raises: ServiceException: When call to rospy fails.

        Example:
            .. code-block:: python

                from problem import Helper, State

                h = Helper()
                execute_status = h.execute_pick_action(book_name, curr_state)
                if execute_status == False:
                    print "Pick Action Failed"

        .. warning::
            This action will fail (Return False) if:

            .. hlist::
                :columns: 1
                
                * Book name is not valid.
                * Robot Location is not within the load location of the book, i.e. robot is not in the viscinity of the book.

        """
        rospy.wait_for_service('execute_pick_action')
        try:
            pick_action = rospy.ServiceProxy('execute_pick_action',PickActionMsg)
            response = pick_action(book_name,state.x,state.y,state.orientation)

            if response.result == -1:
                return False
            elif response.result == 1:
                return True

        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e

    def execute_suck_action(self, dirt_id, state):
        """
        This action is to visualize that dirt has been sucked up by the robot. This action checks if the robot is in the load location of a dirty cell and then removes that dirt from that cell and places it in a remote position
        :param dirt_id: refers to the dirt we attempting to clean
        :param state:the state of the robot
        :return: True if the cleaning was successful, false if not
        """
        rospy.wait_for_service('execute_suck_action')
        try:
            suck_action = rospy.ServiceProsxy('execute_suck_action', SuckActionMsg)
            response = suck_action(dirt_id, state.x, state.y, state.orientation)

            if response.result == -1:
                return False
            elif response.result == 1:
                return True

        except rospy.ServiceException, e:
            print
            "Sevice call failed: %s" % e

    def execute_place_action(self, book_name, bin_name, state):
        """
        This action places the book named book_name in the bin named bin_name when the TurtleBot is at State robot_state.

        :param book_name: Name of the book that has to placed in the bin.
        :type book_name: str
        :param bin_name: Name of the bin where the book named book_name will be placed. 
        :type bin_name: str
        :param robot_state: State of the robot and and the bin.
        :type robot_state: State

        :returns: True if the place action was successful. False, otherwise. 
        :rtype: bool

        :raises: ServiceException: When call to rospy fails.

        Example:
            .. code-block:: python

                from problem import Helper, State

                h = Helper()
                execute_status = h.execute_place_action(book_name, bin_name, curr_state)
                if execute_status == False:
                    print "Pick Action Failed"

        .. warning::
            This action will fail (Return False) if:

            .. hlist::
                :columns: 1
                
                * Book name or Bin name is not valid.
                * Book size and bin size does not match.
                * Subject of the book and bin does not match.
                * Robot Location is not within the load location of the bin, i.e. robot is not in the viscinity of the bin.

        """
        rospy.wait_for_service('execute_place_action')
        try:
            place_action = rospy.ServiceProxy('execute_place_action',PlaceActionMsg)
            response = place_action(book_name,bin_name,state.x,state.y,state.orientation)

            if response.result == -1:
                return False
            elif response.result == 1:
                return True
                        
        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e

    def execute_move_action(self, action_list):
        """
        This action executes the actions in the action_seq. The actions are executed in the same sequence as they appear in action_seq list.

        :param action_seq: Sequence of actions to be executed by the TurtleBot.
        :type action_seq: list(str)

        :rtype: None

        :raises: ServiceException: When call to rospy fails.

        Example:
            .. code-block:: python

                from problem import Helper, State

                h = Helper()
                h.execute_move_action(action_list)

        .. warning::
            This method will execute the actions even if they are wrong semantically, i.e. do not have the desired effect. It is the responsibility of the caller to ensure that the action sequence is applicable from the current state of the robot.

        """
        rospy.wait_for_service('execute_move_action')
        try:
            action_str = "_".join(action for action in action_list)
            move_action = rospy.ServiceProxy("execute_move_action",MoveActionMsg)
            response = move_action(action_str)
            return True
        except rospy.ServiceException,e:
            print "Sevice call failed: %s"%e

    def get_successor(self, state, action):
        rospy.wait_for_service('get_successor')
        try:
            get_successor = rospy.ServiceProxy('get_successor', GetSuccessor)
            response = get_successor(state.x,state.y,state.orientation, action)
            return State(response.x, response.y, response.direction), response.g_cost
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def get_initial_state(self):
        rospy.wait_for_service('get_initial_state')
        try:
            get_initial_state = rospy.ServiceProxy('get_initial_state', GetInitialState)
            response = get_initial_state()
            return State(response.x, response.y, response.direction)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def is_goal_state(self, state):
        rospy.wait_for_service('is_goal_state')
        try:
            is_goal_state_client = rospy.ServiceProxy('is_goal_state', IsGoalState)
            response = is_goal_state_client(state.x,state.y)
            return response.is_goal
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def get_goal_state(self):
        rospy.wait_for_service('get_goal_state')
        try:
            get_goal_state = rospy.ServiceProxy('get_goal_state',GetGoalState)
            response = get_goal_state()
            return State(response.x,response.y,"EAST")
        except rospy.ServiceException,e:
            print "Service call failed: %s"%e

    def get_actions(self):
        return ["TurnCW","TurnCCW","MoveF"]

    def usage(self):
        return "%s [x y]"%sys.argv[0]
