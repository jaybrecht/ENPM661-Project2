import cv2 
import numpy as np

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


def visualize_path(maze_img,nodes,path,scale):
    print('Showing Visualization')
    for i,point in enumerate(nodes):
        if scale == 1:
            maze_img[point[1],point[0]] = (255,255,255)
        else:
            sx = point[0]*scale
            sy = point[1]*scale
            ex = sx+scale
            ey = sy+scale
            cv2.rectangle(maze_img,(sx,sy),(ex,ey),(255,255,255),-1)

        if i%100 == 0:
            cv2.imshow("Maze",maze_img)
            cv2.waitKey(5)

        if cv2.waitKey(1) == ord('q'):
            exit()

    for point in path:
        if scale == 1:
            maze_img[point[1],point[0]] = (0,0,255)
        else:
            sx = point[0]*scale
            sy = point[1]*scale
            ex = sx+scale
            ey = sy+scale
            cv2.rectangle(maze_img,(sx,sy),(ex,ey),(0,0,255),-1)

    cv2.imshow("Maze",maze_img)
    cv2.waitKey(0)
