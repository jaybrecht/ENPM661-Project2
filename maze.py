import cv2 
import math
import numpy as np
from shapely.geometry import Polygon

class Maze:
    def __init__(self,filename,scale):
        # instatiates an object of class maze
       
        self.filename = filename
        self.scale = scale
        self.read_obstacles()

        self.image = np.zeros((self.height*scale,self.width*scale,3),np.uint8)
        self.maze = np.zeros((self.height,self.width),dtype=np.uint8)

        for obs in self.obstacles:
            if obs['type'] == 'c': # circle
                self.draw_circle(obs,0,obs['color'])

            elif obs['type'] == 'p': # polygon
                self.draw_polygon(obs,0,obs['color'])         

            elif obs['type'] == 'e': # ellipse
                self.draw_ellipse(obs,0,obs['color'])

            elif obs['type'] == 'rr': # rotate rect
                self.draw_rotated_rect(obs,0,obs['color'])

        maze_not_scaled = cv2.resize(self.image,(self.width,self.height))
        inds=np.nonzero(maze_not_scaled)
        
        for i in range(len(inds[0])):
            x = inds[1][i]
            y = inds[0][i]
            self.maze[y,x] = 1


    def in_maze(self,point):
        # Checks whether a point is in bounds and not an obstacle
        x = point[0]
        y = point[1]
        if 0<=x<self.width and 0<=y<self.height:
            if self.maze[y,x] != 1:
                return True
        return False

    def draw_circle(self,obs,offset,color):
        # Draws a circle on the maze image
        center = (obs['center'][0]*self.scale,obs['center'][1]*self.scale)
        radius = obs['radius']*self.scale

        self.image = cv2.circle(self.image,center,radius+(offset*self.scale),color,-1)


    def define_circle(self,obs,offset):
        # Write code that modifies that attribute maze to have 1s everywhere inside
        # of the obstacle obs. Should expand the obstacle by the offset


    def draw_polygon(self,obs,offset,color):
        # Draws a polygon on the maze image
        points = []
        for p in obs['points']:
            points.append((p[0]*self.scale,(self.height*self.scale)-p[1]*self.scale))
        
        contour = np.array(points, dtype=np.int32)
        
        if offset == 0:
            self.image = cv2.drawContours(self.image,[contour],-1,color,-1) 
        else:
            off_contour = np.squeeze(contour)
            polygon = Polygon(off_contour)
            offset_poly = polygon.buffer(offset*self.scale,cap_style=2, join_style=2)
            off_points = offset_poly.exterior.coords
            off_contour = np.array(off_points, dtype=np.int32)
            self.image = cv2.drawContours(self.image,[off_contour],-1,color,-1) 


    def define_polygon(self,obs,offset):
        # Write code that modifies that attribute maze to have 1s everywhere inside
        # of the obstacle obs. Should expand the obstacle by the offset


    def draw_ellipse(self,obs,offset,color):
        # Draws an ellipse on the maze image
        center = (obs['center'][0]*self.scale,obs['center'][1]*self.scale)
        
        axis = (obs['axis'][0]*self.scale+(offset*self.scale),
                       obs['axis'][1]*self.scale+(offset*self.scale))
        self.image = cv2.ellipse(self.image, center, axis, obs['angle'],
                        obs['start'], obs['end'],color,-1)


    def define_ellipse(self,obs,offset):
        # Write code that modifies that attribute maze to have 1s everywhere inside
        # of the obstacle obs. Should expand the obstacle by the offset


    def draw_rotated_rect(self,obs,offset,color):
        # Draws a rotated rectangle on the maze image
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
            spoints.append((p[0]*self.scale,(self.height*self.scale)-p[1]*self.scale))
        contour = np.array(spoints, dtype=np.int32)
        
        if offset == 0:
            self.image = cv2.drawContours(self.image,[contour],-1,color,-1) 
        else:
            off_contour = np.squeeze(contour)
            polygon = Polygon(off_contour)
            offset_poly = polygon.buffer(offset*self.scale,cap_style=2, join_style=2)
            off_points = offset_poly.exterior.coords
            off_contour = np.array(off_points, dtype=np.int32)
            self.image = cv2.drawContours(self.image,[off_contour],-1,color,-1)


    def define_rotated_rect(self,obs,offset):
        # Write code that modifies that attribute maze to have 1s everywhere inside
        # of the obstacle obs. Should expand the obstacle by the offset, may be easier
        # to just define the points and pass them into define_polygon


    def expand_obstacles(self,offset):
        off_color = (255,255,255)
        for obs in self.obstacles:
            if obs['type'] == 'c': # circle
                self.draw_circle(obs,offset,off_color)
                self.draw_circle(obs,0,obs['color'])

            elif obs['type'] == 'p': # polygon
                self.draw_polygon(obs,offset,off_color)    
                self.draw_polygon(obs,0,obs['color'])         

            elif obs['type'] == 'e': # ellipse
                self.draw_ellipse(obs,offset,off_color)
                self.draw_ellipse(obs,0,obs['color'])

            elif obs['type'] == 'rr': # rotate rect
                self.draw_rotated_rect(obs,offset,off_color)
                self.draw_rotated_rect(obs,0,obs['color'])

        maze_not_scaled = cv2.resize(self.image,(self.width,self.height))
        self.maze = np.zeros((self.height,self.width),dtype=np.uint8)
        inds=np.nonzero(maze_not_scaled)
        
        for i in range(len(inds[0])):
            x = inds[1][i]
            y = inds[0][i]
            self.maze[y,x] = 1


    def contract_obstacles(self,offset):
        for obs in self.obstacles:
            if obs['type'] == 'c': # circle
                self.draw_circle(obs,offset,0)
                self.draw_circle(obs,0,obs['color'])

            elif obs['type'] == 'p': # polygon
                self.draw_polygon(obs,offset,0)    
                self.draw_polygon(obs,0,obs['color'])         

            elif obs['type'] == 'e': # ellipse
                self.draw_ellipse(obs,offset,0)
                self.draw_ellipse(obs,0,obs['color'])

            elif obs['type'] == 'rr': # rotate rect
                self.draw_rotated_rect(obs,offset,0)
                self.draw_rotated_rect(obs,0,obs['color'])


    def read_obstacles(self):
        # reads in obstacles from a text file. Supports four types of obstacles,
        # circles,polygons,ellipses,and rotatedrects. Returns a list where each
        # obstacle is a dictionary 
        maze_file = open(self.filename,"r")
        lines = maze_file.readlines()
        default_color = (255,0,0)
        obstacles = []

        for i,line in enumerate(lines):
            if line.split(':')[0].strip() == 'height':
                h = line.split(':')[-1]
                self.height = int(h)
            if line.split(':')[0].strip() == 'width':
                w = line.split(':')[-1]
                self.width = int(w)
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
                    if next_line.split(':')[0].strip() == 'l1':
                        height = int(next_line.split(':')[-1])
                    if next_line.split(':')[0].strip() == 'l2':
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
        self.obstacles = obstacles
    

if __name__ == '__main__':
    maze = 'maze2'
    mymaze = Maze(maze+'.txt',3)
    mymaze.expand_obstacles(10)
    cv2.imwrite('Images/expanded_obstacles.png',mymaze.image)
    cv2.waitKey(0)
