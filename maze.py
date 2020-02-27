import cv2 
import math
import numpy as np
from shapely.geometry import Polygon

def init_maze(size,obstacles,scale,offset):
    # init_maze creates a maze of specified size (tuple (w,h)) and obstacles
    # which can be circles, rectangles, polygons, ellipses, or rotated rectangles
    # and returns a scaled image of the maze and a binary numpy array that has the same
    # dimensions as the maze where a '1' represents an obstacle and '0' represents
    # free space
    maze_w = size[0]*scale
    maze_h = size[1]*scale

    maze = np.zeros((maze_h,maze_w,3),np.uint8)

    for obs in obstacles:
        if obs['type'] == 'c': # circle
            center = (obs['center'][0]*scale,obs['center'][1]*scale)
            radius = obs['radius']*scale
            if offset == 0:
                maze = cv2.circle(maze,center,radius,obs['color'],-1)
            else:
                maze = cv2.circle(maze,center,radius+(offset*scale),obs['color'],-1)
                maze = cv2.circle(maze,center,radius,(255,255,255),-1)

        elif obs['type'] == 'p': # polygon
            points = []
            for p in obs['points']:
                points.append((p[0]*scale,maze_h-p[1]*scale))
            contour = np.array(points, dtype=np.int32)
            
            if offset == 0:
                maze = cv2.drawContours(maze,[contour],-1,obs['color'],-1)
            else:
                off_contour = np.squeeze(contour)
                polygon = Polygon(off_contour)
                offset_poly = polygon.buffer(offset*scale,cap_style=2, join_style=2)
                off_points = offset_poly.exterior.coords
                off_contour = np.array(off_points, dtype=np.int32)
                maze = cv2.drawContours(maze,[off_contour],-1,obs['color'],-1) 
                maze = cv2.drawContours(maze,[contour],-1,(255,255,255),-1)           

        elif obs['type'] == 'e': # ellipse
            center = (obs['center'][0]*scale,obs['center'][1]*scale)
            axis_length = (obs['axis'][0]*scale,obs['axis'][1]*scale)

            if offset == 0:
                maze = cv2.ellipse(maze, center, axis_length, obs['angle'],
                    obs['start'], obs['end'],obs['color'],-1)
            else:
                off_axis_length = (obs['axis'][0]*scale+(offset*scale),
                               obs['axis'][1]*scale+(offset*scale))
                maze = cv2.ellipse(maze, center, off_axis_length, obs['angle'],
                                   obs['start'], obs['end'],obs['color'],-1)
                maze = cv2.ellipse(maze, center, axis_length, obs['angle'],
                    obs['start'], obs['end'],(255,255,255),-1)

        elif obs['type'] == 'rr': # rotate rect
            w = obs['width']
            h = obs['height']
            ang1 = math.radians(obs['angle'])
            ang2 = math.radians(90-obs['angle'])
            p1 = obs['start_point']
            p2 = (p1[0]-(w*math.cos(ang1)),p1[1]+(w*math.sin(ang1)))
            p3 = (p1[0]+(h*math.cos(ang2)),p1[1]+(h*math.sin(ang2)))
            p4 = (p3[0]-(w*math.cos(ang1)),p3[1]+(w*math.sin(ang1)))
            points = [p1,p2,p4,p3]
            spoints = []
            for p in points:
                spoints.append((p[0]*scale,maze_h-p[1]*scale))
            contour = np.array(spoints, dtype=np.int32)
            
            if offset == 0:
                maze = cv2.drawContours(maze,[contour],-1,obs['color'],-1)

            else:
                off_contour = np.squeeze(contour)
                polygon = Polygon(off_contour)
                offset_poly = polygon.buffer(offset*scale,cap_style=2, join_style=2)
                off_points = offset_poly.exterior.coords
                off_contour = np.array(off_points, dtype=np.int32)
                maze = cv2.drawContours(maze,[off_contour],-1,obs['color'],-1) 
                maze = cv2.drawContours(maze,[contour],-1,(255,255,255),-1) 

    maze_not_scaled = cv2.resize(maze,(size[0],size[1]))
    maze_arr = np.zeros((size[1],size[0]),dtype=np.int32)
    inds=np.nonzero(maze_not_scaled)
    
    for i in range(len(inds[0])):
        x = inds[1][i]
        y = inds[0][i]
        maze_arr[y][x] = 1

    return maze,maze_arr


def read_obstacles(filename):
    # reads in obstacles from a text file. Supports four types of obstacles,
    # circles,polygons,ellipses,and rotatedrects. Returns a list where each
    # obstacle is a dictionary 
    maze_file = open(filename,"r")
    lines = maze_file.readlines()
    default_color = (255,0,0)
    obstacles = []

    for i,line in enumerate(lines):
        if line == 'circle\n':
            j = i+1
            next_line = lines[j]
            center = radius = None
            color = default_color
            while next_line != '\n':
                if next_line.split(':')[0].strip() == 'center':
                    p = next_line.split(':')[-1].split(',')
                    center = (int(p[0]),int(p[1]))
                if next_line.split(':')[0].strip() == 'radius':
                    radius = int(next_line.split(':')[-1])
                if next_line.split(':')[0].strip() == 'color':
                    bgr = next_line.split(':')[-1].split(',')
                    color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                j += 1
                if j<len(lines):
                    next_line = lines[j]
                else:
                    break

            if radius!=None and center!=None:
                obs = {'type':'c','center':center,'radius':radius,'color':color}
                obstacles.append(obs)
        
        if line == 'polygon\n':
            points = []
            color = default_color
            j = i+1
            next_line = lines[j]
            while next_line != '\n':
                if next_line.split(':')[0].strip() == 'point':
                    p = next_line.split(':')[-1].split(',')
                    points.append((int(p[0]),int(p[1])))
                if next_line.split(':')[0].strip() == 'color':
                    bgr = next_line.split(':')[-1].split(',')
                    color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                j += 1
                if j<len(lines):
                    next_line = lines[j]
                else:
                    break

            if len(points)>0:
                obs = {'type':'p','points':points,'color':color}
                obstacles.append(obs)
        
        if line == 'ellipse\n': 
            center = axis = None
            angle = start = 0
            end = 360
            color = default_color

            j = i+1
            next_line = lines[j]

            while next_line != '\n':
                if next_line.split(':')[0].strip() == 'center':
                    p = next_line.split(':')[-1].split(',')
                    center = (int(p[0]),int(p[1]))
                if next_line.split(':')[0].strip() == 'axis':
                    p = next_line.split(':')[-1].split(',')
                    axis = (int(p[0]),int(p[1]))
                if next_line.split(':')[0].strip() == 'angle':
                    angle = int(next_line.split(':')[-1])
                if next_line.split(':')[0].strip() == 'start':
                    radius = int(next_line.split(':')[-1])
                if next_line.split(':')[0].strip() == 'end':
                    radius = int(next_line.split(':')[-1])
                if next_line.split(':')[0].strip() == 'color':
                    bgr = next_line.split(':')[-1].split(',')
                    color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                j += 1
                if j<len(lines):
                    next_line = lines[j]
                else:
                    break
            
            if radius!=None and axis!=None:
                obs = {'type':'e','center':center,'axis':axis,'angle':angle,
                       'start':start,'end':end,'color':color}
                obstacles.append(obs)

        if line == 'rotatedrect\n': 
            start_point = height = width = angle = None
            color = default_color

            j = i+1
            next_line = lines[j]

            while next_line != '\n':
                if next_line.split(':')[0].strip() == 'start_point':
                    p = next_line.split(':')[-1].split(',')
                    start_point = (int(p[0]),int(p[1]))
                if next_line.split(':')[0].strip() == 'height':
                    height = int(next_line.split(':')[-1])
                if next_line.split(':')[0].strip() == 'width':
                    width = int(next_line.split(':')[-1])
                if next_line.split(':')[0].strip() == 'angle':
                    angle = int(next_line.split(':')[-1])
                if next_line.split(':')[0].strip() == 'color':
                    bgr = next_line.split(':')[-1].split(',')
                    color = (int(bgr[0]),int(bgr[1]),int(bgr[2]))
                j += 1
                if j<len(lines):
                    next_line = lines[j]
                else:
                    break
            
            if start_point!=None and height!=None and width!=None and angle!=None:
                obs = {'type':'rr','start_point':start_point,'height':height,
                       'width':width,'angle':angle,'color':color}
                obstacles.append(obs)

    maze_file.close()

    return obstacles

if __name__ == '__main__':
    maze1_obs = read_obstacles('maze1.txt')
    maze2_obs = read_obstacles('maze2.txt')

    maze1_img,matrix1 = init_maze([200,100],maze1_obs,3,10)
    maze2_img,matrix2 = init_maze([300,200],maze2_obs,3,5)

    cv2.imshow("Maze2",maze2_img)

    cv2.waitKey(0)
