import cv2 
import numpy as np
import math
from collections import deque
from maze import*

def in_maze(point,maze):
    # Checks whether a point is in bounds and not an obstacle
    x = point[0]
    y = point[1]
    if 0<=x<maze.shape[1] and 0<=y<maze.shape[0]:
        if maze[y,x] != 1:
            return True
    return False

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
    cur_point = cur_node[0]
    directions = ['E','NE','N','NW','W','SW','S','SE']

    neighbors = []
    for direction in directions:
        new_point = move(cur_point,direction)
        if in_maze(new_point,maze):
            if len(direction) == 2:
                cost = math.sqrt(2)
            else:
                cost = 1

            neighbors.append((new_point,cost))

    return neighbors


def generate_path(nodes):
    #Assume the last item in nodes is the goal node
    parent = nodes[-1][1]
    path_nodes = [parent]
    while parent != -1:
        parent_node = nodes[path_nodes[-1]]
        parent = parent_node[1]
        path_nodes.append(parent)
    path = [nodes[-1][0]]
    for ind in path_nodes:
        if ind == -1:
            break
        else:
            path.insert(0,nodes[ind][0])
    return path

def BFS(start_point,goal_point,maze):
    nodes = [[]]

    # Checked points are additionally stored in a set which is much faster for 
    # checking if the node has already been visited
    points = {start_point}

    start_node = [start_point,-1,0] #set start node to have parent of -1 and cost of 0
    nodes[0] = (start_node) #add the start node to nodes

    # The queue is strucuted as a deque which allows for much faster operation
    # when accessing the first item in the list
    queue = deque()
    queue.append(0) #set the start_node as the first node in the queue

    isgoal = False
    success = False

    while queue:
        # Set the current node as the top of the queue and remove it
        parent = queue.popleft()
        cur_node = nodes[parent]
        neighbors = check_neighbors(cur_node,maze)

        for neighbor in neighbors:
            if neighbor[0] not in points:
                new_node = [neighbor[0],parent,neighbor[1]]
                points.add(neighbor[0])
                nodes.append(new_node)
                queue.append(len(nodes)-1)
            if neighbor[0] == goal_point:
                isgoal = True
                break
        if isgoal:
            success = True
            break

    return [nodes,success]

if __name__ == '__main__':
    maze2_obs = read_obstacles('maze2.txt')
    maze1_img,maze_arr2 = init_maze([300,200],maze2_obs,1)

    nodes,success = BFS((0,0),(299,199),maze_arr2)
    path = generate_path(nodes)
    for point in path:
        maze1_img[point[1],point[0]] = (255,255,255)

    cv2.imshow("Completed Maze",maze1_img)
    cv2.waitKey(0)

