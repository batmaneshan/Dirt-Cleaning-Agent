#!/usr/bin/env python
# encoding: utf-8

import heapq
import environment_api as api 
import rospy
from std_msgs.msg import String
import numpy as np
import random
import json
class RandomWalker:

    def __init__(self, grid_dimension, dirt_locations):
        actions = {}
        action_list = ["UP", "DOWN", "LEFT", "RIGHT", "CLEAN"]
        for action in action_list:
            possible_actions = []
            if(action == "UP" or action == "DOWN"):
                possible_actions.append("LEFT")
                possible_actions.append("RIGHT")
            elif(action == "LEFT" or action == "RIGHT"):
                possible_actions.append("UP")
                possible_actions.append("DOWN")
            actions[action] = {}
            actions[action]["ActionProbabilities"] = {}
            actions[action]["ActionProbabilities"][action] = 0.8
            for possible_action in possible_actions:
                actions[action]["ActionProbabilities"][possible_action] = 0.1
                
        actions["CLEAN"] = {}
        actions["CLEAN"]["ActionProbabilities"] = {}
        actions["CLEAN"]["ActionProbabilities"]["CLEAN"] = 1.0
        self.actions = actions
        self.action_to_index_mapping = {"UP":1, "DOWN":2, "LEFT":3, "RIGHT":4 , "CLEAN":5}
        self.index_to_action_mapping = {1:"UP", 2:"DOWN", 3:"LEFT", 4:"RIGHT", 5:"CLEAN"}
        self.grid_dimension = grid_dimension
        self.dirt_locations = dirt_locations
        self.reward_for_cleaning_a_dirt = 25
        self.count_of_dirty_cells = len(dirt_locations)
        self.gamma = 0.7
        self.main()

    def generate_reward_metric(self):
        reward = np.zeros((self.grid_dimension, self.grid_dimension))
        for dirt_location in self.dirt_locations:
            row,col = dirt_location
            reward[row][col] = self.reward_for_cleaning_a_dirt
        return reward    

    def get_next_cell_on_action(self, current_position, action):
        next_row, next_col = current_position
        if(action == "UP"):
            next_row -=1
            if(next_row < 0):
                next_row+=1
        elif(action == "DOWN"):
            next_row +=1
            if(next_row >= self.grid_dimension):
                next_row-=1
        elif(action == "LEFT"):
            next_col -=1
            if(next_col < 0):
                next_col+=1
        elif(action == "RIGHT"):
            next_col +=1
            if(next_col >= self.grid_dimension ):
                next_col-=1
        return next_row, next_col

    def execute_action(self, bot_location, action):
        #action is a number 
        action = self.index_to_action_mapping[action]
        
        random_number = random.random()
        possibilities = self.actions[action]["ActionProbabilities"]
        cumulative_value = 0
        for possibility in possibilities.keys():
            cumulative_value += possibilities[possibility]
            if(random_number <= cumulative_value):
                return True, self.get_next_cell_on_action(bot_location, possibility)

    def get_max_over_all_actions(self, row, col, V_Values):
        maximum_V = -1
        maximum_rewarding_action = None
        reward = self.generate_reward_metric()
        for action in self.actions.keys():
            possible_actions = self.actions[action]["ActionProbabilities"]
            value_computed = 0
            for possible_action in possible_actions.keys():
                next_state_row, next_state_col = self.get_next_cell_on_action((row,col), possible_action)
                value_computed = value_computed + possible_actions[possible_action]*(reward[row][col] + self.gamma*V_Values[next_state_row][next_state_col])
       
            if(value_computed > maximum_V):
                maximum_V = value_computed
                maximum_rewarding_action = action
        return maximum_V, self.action_to_index_mapping[maximum_rewarding_action]

    def Value_Iteration(self, V_Values):
        policy = np.zeros((self.grid_dimension, self.grid_dimension))
        Final_V_Values = np.zeros((self.grid_dimension, self.grid_dimension))
        theta = 0.1
        
        while(True):
            delta = 0
            for row in range(self.grid_dimension):
                for col in range(self.grid_dimension):
                    if((row,col) in self.dirt_locations):
                        Final_V_Values[row][col], maximising_action = self.reward_for_cleaning_a_dirt,5
                    else:
                        Final_V_Values[row][col], maximising_action = self.get_max_over_all_actions(row,col,V_Values)
                    policy[row][col] = maximising_action
                    delta = max(delta, np.abs(Final_V_Values[row][col] - V_Values[row][col]))
            
            for row in range(self.grid_dimension):
                for col in range(self.grid_dimension):
                    V_Values[row][col] = Final_V_Values[row][col]
            if(delta < theta):
                break
        return Final_V_Values, policy
    
    def Policy_Evaluation(self, policy, initial_V_Values):
        # return 2d array of values
        # policy is a 2d array containing the action for each cell
        Final_V_Values = np.zeros((self.grid_dimension, self.grid_dimension))
        reward = self.generate_reward_metric()
        theta = 0.1
        while (True):
            delta = 0
            # print(initial_V_Values)
            for row in range(self.grid_dimension):
                for col in range(self.grid_dimension):
                    if((row,col) in self.dirt_locations):
                            Final_V_Values[row][col] = self.reward_for_cleaning_a_dirt
                    
                    else:
                        action_as_per_policy = policy[row][col]
                        action_as_per_policy = self.index_to_action_mapping[action_as_per_policy]
                        possible_actions = self.actions[action_as_per_policy]["ActionProbabilities"]
                        for possible_action in possible_actions.keys():
                            next_state_row, next_state_col = self.get_next_cell_on_action((row,col), possible_action)
                            Final_V_Values[row][col] = Final_V_Values[row][col] + possible_actions[possible_action]*(reward[row][col] + self.gamma*initial_V_Values[next_state_row][next_state_col])

                    delta = max(delta, np.abs(Final_V_Values[row][col] - initial_V_Values[row][col]))

            for row in range(self.grid_dimension):
                for col in range(self.grid_dimension):
                    initial_V_Values[row][col] = Final_V_Values[row][col]
                    Final_V_Values[row][col] = 0

            if(delta < theta):
                break
        return initial_V_Values

    def Policy_Iteration(self, initial_V_Values, policy):
        # return converged policy
        updated_policy = np.zeros((self.grid_dimension, self.grid_dimension))
        while(True):
            policy_updated = False
            next_V_Values = self.Policy_Evaluation(policy, initial_V_Values)
            for row in range(self.grid_dimension):
                for col in range(self.grid_dimension):
                    if((row,col) in self.dirt_locations):
                        updated_policy[row][col] = 5
                    else:
                        temp, updated_policy[row][col] = self.get_max_over_all_actions(row, col, next_V_Values)
                    if(updated_policy[row][col] - policy[row][col] !=0):
                        policy_updated = True
                        policy[row][col] = updated_policy[row][col]
            for row in range(self.grid_dimension):
                for col in range(self.grid_dimension):
                    initial_V_Values[row][col] = next_V_Values[row][col]
            if(policy_updated == False):
                break
        return policy, initial_V_Values
    
    def main(self):

        # compute a policy based on the initial configuration 
        print("initial dirt locations")
        print(self.dirt_locations)
        print("initial count of dirty cells "+ str(self.count_of_dirty_cells))
        # V_Values = np.zeros((self.grid_dimension, self.grid_dimension))
        # Updated_V_Values, policy = self.Value_Iteration(V_Values)
        
        # print("initial policy")
        # print(policy)

        bot_current_location = (0,0)
        total_steps_of_cleaning_entire_grid = 0
        
        while(self.count_of_dirty_cells > 0):
            # get bot's location from helper function
            if(bot_current_location in self.dirt_locations):
                isSuccess, next_state = self.execute_action(bot_current_location, 5)
                if(isSuccess == True):
                    print("taking action ", self.index_to_action_mapping[5], "next cell: ",next_state)
                    self.count_of_dirty_cells = self.count_of_dirty_cells - 1
                    self.dirt_locations.remove(bot_current_location)
                    if(self.count_of_dirty_cells <= 0):
                        total_steps_of_cleaning_entire_grid = total_steps_of_cleaning_entire_grid + 1
                        break
                    # policy, Updated_V_Values = self.Policy_Iteration(Updated_V_Values, policy)
                    # print("updated policy")
                    # print(policy)
                    # print("updated v values")
                    # print(Updated_V_Values)
                else:
                    print("ERROR")
                    return
            else:
                # follow the policy and execute the best action from current state
                bot_current_location_row, bot_current_location_col = bot_current_location
                # best_action = policy[bot_current_location_row][bot_current_location_col]
                best_action = random.randint(1,4)
                isSuccess, next_state = self.execute_action(bot_current_location, best_action)
                print("taking action ", self.index_to_action_mapping[best_action], "next cell: ",next_state)

            total_steps_of_cleaning_entire_grid = total_steps_of_cleaning_entire_grid + 1
            print("step number : " + str(total_steps_of_cleaning_entire_grid))
            print("count of dirty cells "+ str(self.count_of_dirty_cells))
            bot_current_location = next_state
            
        print("total_steps: " + str(total_steps_of_cleaning_entire_grid))

if __name__ == "__main__":
    json_file = open('../objects.json')
    data = json.load(json_file)
    grid_dimension = data["grid_size"]

    dirts = data["dirts"]
    dirt_locations = []
    for dirt in dirts.keys():
        dirt_loc = dirts[dirt]["loc"]
        dirt_loc = int(dirt_loc[0]),int(dirt_loc[1])
        dirt_locations.append(dirt_loc)
    
    RandomWalker(grid_dimension, dirt_locations)