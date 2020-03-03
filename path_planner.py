import cv2 
import numpy as np
import time
import math

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


def visualize_path(maze_img,nodes,path,scale,robot_size):
    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    frame_size = (maze_img.shape[1], maze_img.shape[0])
    today = time.strftime("%m-%d__%H.%M.%S")
    videoname=str(today)
    fps_out = 60
    out = cv2.VideoWriter(str(videoname)+".mp4", fourcc, fps_out, frame_size)
    cur_frame = 1
    print("Writing to Video, Please Wait")
    stepsize = 50
    tot_frames = (len(nodes)//stepsize)+1
  
    for i,point in enumerate(nodes):
        if scale == 1:
            maze_img[point[1],point[0]] = (0,255,255)
        else:
            sx = point[0]*scale
            sy = point[1]*scale
            ex = sx+scale
            ey = sy+scale
            cv2.rectangle(maze_img,(sx,sy),(ex,ey),(0,255,255),-1)

        if i%stepsize == 0:
            print('Frame number:' + str(cur_frame) + ' of ' + str(tot_frames))
            out.write(maze_img)
            time.sleep(0.005)
            cur_frame += 1


    for point in path:
        sx = point[0]*scale
        sy = point[1]*scale
        cv2.circle(maze_img,(sx,sy),robot_size*scale,(0,0,255),-1)
        out.write(maze_img)
        time.sleep(0.005)

    out.release()
