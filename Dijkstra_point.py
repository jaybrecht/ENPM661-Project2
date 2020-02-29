import cv2 
import numpy as np
import math
from collections import deque
from maze import*
from path_planner import*

def move(point,direction):
    x = point[0]
    y = point[1]
    if direction == 'E':
        new_point = (x-1,y)
    elif direction == 'NE':
        new_point = (x-1,y+1)
    elif direction == 'N':
        new_point = (x,y+1)
    elif direction == 'NW':
        new_point = (x+1,y+1)
    elif direction == 'W':
        new_point = (x+1,y)
    elif direction == 'SW':
        new_point = (x+1,y-1)
    elif direction == 'S':
        new_point = (x,y-1)
    elif direction == 'SE':
        new_point = (x,y-1)

    return new_point


def check_neighbors(cur_node,maze):
    directions = ['E','NE','N','NW','W','SW','S','SE']

    neighbors = []
    for direction in directions:
        new_point = move(cur_node,direction)
        if in_maze(new_point,maze):
            if len(direction) == 2:
                cost = math.sqrt(2)
            else:
                cost = 1

            neighbors.append((new_point,cost))

    return neighbors


def BFS(start_point,goal_point,maze):
    nodes = []

    # Checked points are additionally stored in a set which is much faster for 
    # checking if the node has already been visited
    points = {start_point}
    parents = np.full((maze.shape[0],maze.shape[1]),np.nan,dtype=np.int32)

     #set start node to have parent of -1 and cost of 0
    nodes.append(start_point) #add the start node to nodes
    parents[start_point[1],start_point[0]] = -1

    # The queue is strucuted as a deque which allows for much faster operation
    # when accessing the first item in the list
    queue = deque()
    queue.append(0) #set the start_node as the first node in the queue

    isgoal = False

    while queue:
        # Set the current node as the top of the queue and remove it
        parent = queue.popleft()
        cur_node = nodes[parent]
        neighbors = check_neighbors(cur_node,maze)

        for n in neighbors:
            p = n[0]
            if p not in points:
                nodes.append(p)
                points.add(p)
                queue.append(len(nodes)-1)
                parents[p[1],p[0]] = parent
            if p == goal_point:
                isgoal = True
                queue.clear()
                break

    return [nodes,parents,isgoal]

def Dijkstra(start_point,goal_point,maze):
    nodes = []

    # Checked points are additionally stored in a set which is much faster for 
    # checking if the node has already been visited
    points = {start_point}
    costs = np.full((maze.shape[0],maze.shape[1]),np.inf)
    parents = np.full((maze.shape[0],maze.shape[1]),np.nan,dtype=np.int32)

     #set start node to have parent of -1 and cost of 0
    nodes.append(start_point) #add the start node to nodes
    costs[start_point[1],start_point[0]] = 0
    parents[start_point[1],start_point[0]] = -1

    # The queue is strucuted as a deque which allows for much faster operation
    # when accessing the first item in the list
    queue = deque()
    queue.append(0) #set the start_node as the first node in the queue

    isgoal = False
    cost2come = 0

    while queue:
        # Set the current node as the top of the queue and remove it
        parent = queue.popleft()
        cur_node = nodes[parent]
        cost2come = costs[cur_node[1],cur_node[0]]
        neighbors = check_neighbors(cur_node,maze)

        for n in neighbors:
            p = n[0]
            c = n[1]
            if p not in points:
                nodes.append(p)
                points.add(p)
                queue.append(len(nodes)-1)
            if cost2come + c < costs[p[1],p[0]]:
                costs[p[1],p[0]] = cost2come + c
                parents[p[1],p[0]] = parent
            if p == goal_point:
                isgoal = True
                queue.clear()
                break

    return [nodes,parents,costs,isgoal]



if __name__ == '__main__':
    maze_obs = read_obstacles('maze1.txt')
    maze_size = (200,100)
    robot_size = 3
    scale = 1
    maze_img,maze_arr = init_maze(maze_size,maze_obs,scale,robot_size)

    start_point = (0,0)
    goal_point = (199,99)
    if in_maze(goal_point,maze_arr):
        nodes,parents,costs,isgoal = Dijkstra(start_point,goal_point,maze_arr)
        if isgoal:
            print('The goal has been found')
            path = generate_path(nodes,parents)
            print('The path has been found')
            visualize_path(maze_img,nodes,path,scale,robot_size)
        else:
            print('The goal cannot be reached')
    else:
        print('The goal point is not valid')



