# -*- coding: utf-8 -*-
"""
Created on Sat Feb 11 22:50:50 2017

@author: Farrukh Jalali
"""

import numpy as np
import sys
import time

# Generic Tree object will act as an artifact for different search techniques

class Tree(object):
    def __init__(self, state, depth, name="root", parent=""):
        self.name = name
        self.state = state
        self.childern = []
        self.tree_depth = depth
        self.parent = parent

    def __repr__(self):
        return self.state
    
    def add_child(self, node):
        assert isinstance(node,Tree)
        self.childern.append(node)
    
    def mutate_state(self, i, j ): 
        l = list(self.state)
        l[i], l[j] = l[j], l[i]
        return "".join(l)
        
    def generate_childern(self, wd, ht):
        pos = self.state.index('0')
        x = (pos+1)%ht
        
        if (pos - wd) >= 0 : #checking for legible UP move
            childState = self.mutate_state(pos, pos-wd)
            self.add_child(Tree(childState, self.tree_depth+1, name="Up", parent=self))
        
        if (pos + wd) < (wd * ht): #checking for legible DOWN move
            childState = self.mutate_state(pos, pos+wd)
            self.add_child(Tree(childState, self.tree_depth+1, name="Down", parent=self))

        if x != 1: #checking for legible LEFT move
            childState = self.mutate_state(pos, pos-1)
            self.add_child(Tree(childState, self.tree_depth+1, name="Left", parent=self))

        if x != 0: #checking for legible RIGHT move
            childState = self.mutate_state(pos, pos+1)
            self.add_child(Tree(childState, self.tree_depth+1, name="Right", parent=self))

class extendedTree(Tree):
    def __init__(self, state, depth, name="root", parent="", g=0, h=0):
        Tree.__init__(self,state, depth, name, parent)
        self.g = g
        self.h = h
        
    def cost(self):
        return 1 if self.state else 0

    def generate_childern(self, wd, ht):
        pos = self.state.index('0')
        x = (pos+1)%ht
        
        if (pos - wd) >= 0 : #checking for legible UP move
            childState = self.mutate_state(pos, pos-wd)
            self.add_child(extendedTree(childState, self.tree_depth+1, name="Up", parent=self))
        
        if (pos + wd) < (wd * ht): #checking for legible DOWN move
            childState = self.mutate_state(pos, pos+wd)
            self.add_child(extendedTree(childState, self.tree_depth+1, name="Down", parent=self))

        if x != 1: #checking for legible LEFT move
            childState = self.mutate_state(pos, pos-1)
            self.add_child(extendedTree(childState, self.tree_depth+1, name="Left", parent=self))

        if x != 0: #checking for legible RIGHT move
            childState = self.mutate_state(pos, pos+1)
            self.add_child(extendedTree(childState, self.tree_depth+1, name="Right", parent=self))

def ida(initialState, goalTest):   #Iterative Deepning A*
    start_time = time.clock()

    max_fringe = 0
    max_depth = 0
    
    goal_x = (goalTest.index('0')+1) / 3
    goal_y = (goalTest.index('0')+1) % 3
    
    ini_x = (initialState.state.index('0')+1) / 3
    ini_y = (initialState.state.index('0')+1) % 3

    bound = abs(ini_x - goal_x) + abs(ini_y - goal_y)
    
    while 1:
        frontier = []
        frontier.append(initialState)
        explored = 0
        
        while frontier:
            state = frontier.pop()
            explored += 1
            
            max_depth = max(max_depth, state.tree_depth)
            
            if state.state == goalTest:
                depth = state.tree_depth
                path = []
                while state.parent:
                    path.append(state.name)
                    state = state.parent
                path.reverse()
                path_len = len(path)
                result = dict({"path_to_goal": path, "cost_of_path": path_len, "nodes_expanded":explored,"fringe_size":len(frontier),"max_fringe_size":max_fringe,"search_depth":depth-1,"max_search_depth":max_depth,"running_time":(time.clock()-start_time),"max_ram_usage":10000})
                return result
    
            if state.tree_depth > bound:  #check whether depth has reached the current bound 
                bound = state.tree_depth
                continue
           
            state.generate_childern(3,3)
            
            for child in reversed(state.childern):
                frontier.append(child)
            
            max_fringe = max(max_fringe, len(frontier))
            
# A Star Search Techique

def ast(initialState, goalTest): # A Star
    start_time = time.clock()
    frontier = []
    explored = set()
    fringe = set()
    max_fringe = 0
    max_depth = 0
    
#    goal_cord = np.asarray(np.where(goalTest==goalTest.min())).T
    pos = goalTest.index('0')
    goal_x = pos / 3
    goal_y = pos % 3
    
    frontier.append(initialState)
    fringe.add(initialState.state)
    max_fringe = max(max_fringe,len(frontier))   
    
    while frontier:
        state = min(frontier, key=lambda x: x.g + x.h)
        if state.state == goalTest:
            depth = state.tree_depth
            path = []
            while state.parent:
                path.append(state.name)
                state = state.parent
            path.reverse()
            nodes_expanded = len(explored)-1
            path_len = len(path)
            result = dict({"path_to_goal": path, "cost_of_path": path_len, "nodes_expanded":nodes_expanded,"fringe_size":len(frontier),"max_fringe_size":max_fringe,"search_depth":depth-1,"max_search_depth":max_depth,"running_time":(time.clock()-start_time),"max_ram_usage":10000})
            return result
            
        frontier.remove(state)
        fringe.discard(state.state)
        
        explored.add(state.state)
        
        state.generate_childern(3,3)
        
        for child in state.childern:
            if child.state in explored:
                continue  # ignore this child as already evaluated and pick up the next one for evaluation
            if child.state in fringe: # if already waiting in a queue
                gDash = state.g + state.cost()
                if child.g > gDash:
                    child.g = gDash
                    child.parent = state
            else:  # New in town
                child.g = state.g + state.cost()
                pos = child.state.index('0')
                x = pos / 3
                y = pos % 3
                child.h = abs(x - goal_x) + abs(y - goal_y) 
                child.parent = state
                
                frontier.append(child)
                max_depth = max(max_depth, child.tree_depth)
                max_fringe = max(max_fringe,len(frontier))
                fringe.add(child.state)

# Breadth First Search Algorithm

def bfs(initialState, goalTest):  #Breadth First Search
    start_time = time.clock()
    frontier = []
    explored = set()
    fringe = set()

    frontier.append(initialState)
    fringe.add(initialState.state)
    max_fringe = 0
    max_depth = 0
    while frontier:
        state = frontier.pop(0)
        fringe.discard(state.state)
        explored.add(state.state)
        
        if (goalTest==state.state):
            #print("successful")
            depth = state.tree_depth
            path = []
            while state.parent:
                path.append(state.name) 
                state = state.parent
            path.reverse()
            nodes_expanded = len(explored)-1
            path_len = len(path)
            result = dict({"path_to_goal": path, "cost_of_path": path_len, "nodes_expanded":nodes_expanded,"fringe_size":len(frontier),"max_fringe_size":max_fringe,"search_depth":depth-1,"max_search_depth":max_depth,"running_time":(time.clock()-start_time),"max_ram_usage":10000})
            return result
        
        state.generate_childern(3,3)

        for child in state.childern:
            is_explored = False
            
            if child.state in explored:
                is_explored = True
            if not is_explored:
                if child.state in fringe:
                    is_explored = True
            if not is_explored:
                frontier.append(child)
                fringe.add(child.state)
                max_depth = max(max_depth, child.tree_depth)
                max_fringe = max(max_fringe, len(frontier))
    return ""

#Depth First Search

def dfs(initialState, goalTest):  
    start_time = time.clock()
    frontier = []
    explored = set()
    fringe = set()
    
    frontier.append(initialState)
    max_fringe = 0
    max_depth = 0
    while frontier:
        state = frontier.pop()
        fringe.discard(state.state)
        explored.add(state.state)
        
        if (goalTest==state.state):
            #print("successful")
            depth = state.tree_depth
            path = []
            while state.parent:
                path.append(state.name) 
                state = state.parent
            path.reverse()
            nodes_expanded = len(explored)-1
            path_len = len(path)
            result = dict({"path_to_goal": path, "cost_of_path": path_len, "nodes_expanded":nodes_expanded,"fringe_size":len(frontier),"max_fringe_size":max_fringe,"search_depth":depth-1,"max_search_depth":max_depth,"running_time":(time.clock()-start_time),"max_ram_usage":10000})
            return result
        
        state.generate_childern(3,3)
        
        for child in reversed(state.childern):
            is_explored = False
            
            if child.state in explored:
                is_explored = True
            if not is_explored:
                if child.state in fringe:
                    is_explored = True
            if not is_explored:
                frontier.append(child)
                fringe.add(child.state)
                max_depth = max(max_depth, child.tree_depth)
                max_fringe = max(max_fringe, len(frontier))
    return ""

# Writing the result to the file    

def writeFile(d):
    file = open("output.txt","w")
    string = "path_to_goal: " + str(d['path_to_goal']) + "\n"
    file.write(string)
    string = "cost_of_path: " + str(d['cost_of_path']) + "\n"
    file.write(string)
    string = "nodes_expanded: " + str(d['nodes_expanded']) + "\n"
    file.write(string)
    string = "fringe_size: " + str(d['fringe_size']) + "\n"
    file.write(string)
    string = "max_fringe_size: " + str(d['max_fringe_size']) + "\n"
    file.write(string)
    string = "search_depth: " + str(d['search_depth']) + "\n"
    file.write(string)
    string = "max_search_depth: " + str(d['max_search_depth']) + "\n"
    file.write(string)
    string = "running_time: " + str(d['running_time']) + "\n"
    file.write(string)
    string = "max_ram_usage: " + str(d['max_ram_usage']) + "\n"
    file.write(string)
    file.close()

'''
Main function to solve the rubik number puzzle. Program will be called from command line with two parameters : 1- method (i.e. bfs, dfs, ast, ida) 2 - puzzle as a string of 9 digits to be solved. Sample call from command line looks like this:

python driver.py dfs '014235678'
'''
  
def main():
    goal = "012345678"

    method = sys.argv[1]
    board = sys.argv[2]
    
    iniBoard = board.replace(",","")
    
    if method == "ast":
        tree = extendedTree(iniBoard, 1)
    else:
        tree = Tree(iniBoard, 1)
    
    res = globals()[method](tree, goal)
    
    if res:        
        writeFile(res)

if __name__ == "__main__":
    main()
