import cv2
import math
import numpy as np
np.set_printoptions(threshold=np.inf)

def init_maze(size,obstacles,scale):

    maze_w = size[0]*scale
    maze_h = size[1]*scale

    maze = np.zeros((maze_h,maze_w,3),np.uint8)

    for obs in obstacles:
        if obs[0] == 'c': # circle
            center = (obs[1][0]*scale,obs[1][1]*scale)
            radius = obs[2]*scale
            maze = cv2.circle(maze,center,radius,(255,0,0),-1)
        elif obs[0] == 'r': # rectangle
            p1 = (obs[1][0]*scale,obs[1][1]*scale)
            p2 = (obs[2][0]*scale,obs[2][1]*scale)
            maze = cv2.rectangle(maze,p1,p2,(255,0,0),-1)
        elif obs[0] == 'p': # polygon
            points = []
            for p in obs[1]:
                points.append((p[0]*scale,maze_h-p[1]*scale))
            contour = np.array(points, dtype=np.int32)
            maze = cv2.drawContours(maze,[contour],-1,(255,0,0),-1)
        elif obs[0] == 'e': # ellipse
            center = (obs[1][0]*scale,obs[1][1]*scale)
            axis_length = (obs[2][0]*scale,obs[2][1]*scale)
            maze = cv2.ellipse(maze, center, axis_length, 0, 0, 360, (255,0,0),-1)
        elif obs[0] == 'rr': # rotate rect
            w = obs[2][0]
            h = obs[2][1]
            ang1 = math.radians(obs[3])
            ang2 = math.radians(90-obs[3])
            p1 = obs[1]
            p2 = (p1[0]-(w*math.cos(ang1)),p1[1]+(w*math.sin(ang1)))
            p3 = (p1[0]+(h*math.cos(ang2)),p1[1]+(h*math.sin(ang2)))
            p4 = (p3[0]-(w*math.cos(ang1)),p3[1]+(w*math.sin(ang1)))
            points = [p1,p2,p4,p3]
            spoints = []
            for p in points:
                spoints.append((p[0]*scale,maze_h-p[1]*scale))
            contour = np.array(spoints, dtype=np.int32)
            maze = cv2.drawContours(maze,[contour],-1,(255,0,0),-1)

    maze_not_scaled = cv2.resize(maze,(size[0],size[1]))
    matrix = np.zeros((size[1],size[0]),dtype=np.int32)
    inds=np.nonzero(maze_not_scaled)
    for i in range(len(inds[0])):
        x = inds[1][i]
        y = inds[0][i]
        matrix[y][x] = 1

    return maze,matrix

if __name__ == '__main__':
    obs1 = ['r',(90,40),(110,60)]
    obs2 = ['c',(160,50),15]

    maze1_obs = [obs1,obs2]

    obs3 = ['c',(225,50),25]
    obs4 = ['p',[(25,185),(75,185),(100,150),(75,120),(50,150),(20,120)]]
    obs5 = ['p',[(225,10),(250,25),(225,40),(200,25)]]
    obs6 = ['e',(150,100),(40,20)]
    obs7 = ['rr',(95,30),(75,10),30]

    maze2_obs = [obs3,obs4,obs5,obs6,obs7]

    maze1_img,matrix1 = init_maze([200,100],maze1_obs,3)
    maze2_img,matrix2 = init_maze([300,200],maze2_obs,3)
    
    cv2.imshow("Maze1",maze1_img)
    cv2.imshow("Maze2",maze2_img)
    cv2.waitKey(0)
