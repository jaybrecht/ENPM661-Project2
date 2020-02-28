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


def generate_path(nodes,parents):
    #Assume the last item in nodes is the goal node
    goal = nodes[-1]
    parent = parents[goal[1],goal[0]]
    path_nodes = [parent]
    while parent != -1:
        parent_node = nodes[path_nodes[-1]]
        parent = parents[parent_node[1],parent_node[0]]
        path_nodes.append(parent)
    path = [goal]
    for ind in path_nodes:
        if ind == -1:
            break
        else:
            path.insert(0,nodes[ind])
    return path

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
    maze2_obs = read_obstacles('maze2.txt')
    maze_size = (300,200)
    scale = 10
    offset = 0
    maze1_img_d,maze_arr = init_maze(maze_size,maze2_obs,scale,offset)
    maze1_img_b = maze1_img_d.copy()

    start_point = (0,0)
    goal_point = (150,150)
    if in_maze(goal_point,maze_arr):
        d_nodes,d_parents,costs,d_isgoal = Dijkstra(start_point,goal_point,maze_arr)
        b_nodes,b_parents,b_isgoal = BFS(start_point,goal_point,maze_arr)
    else:
        d_isgoal = False
        b_isgoal = False

    if d_isgoal:
        path = generate_path(d_nodes,d_parents)
        for point in path:
            if scale == 1:
                maze1_img_d[point[1],point[0]] = (255,255,255)
            else:
                sx = point[0]*scale
                sy = point[1]*scale
                ex = sx+scale
                ey = sy+scale
                cv2.rectangle(maze1_img_d,(sx,sy),(ex,ey),(255,255,255),-1)

    if b_isgoal:
        path = generate_path(b_nodes,b_parents)
        for point in path:
            if scale == 1:
                maze1_img_b[point[1],point[0]] = (255,255,255)
            else:
                sx = point[0]*scale
                sy = point[1]*scale
                ex = sx+scale
                ey = sy+scale
                cv2.rectangle(maze1_img_b,(sx,sy),(ex,ey),(255,255,255),-1)
        
    # else:
    #     print("The goal can not be reached")
    
    cv2.imshow("Completed Dijkstra Maze",maze1_img_d)
    cv2.imshow("Completed BFS Maze",maze1_img_b)
    cv2.waitKey(0)


