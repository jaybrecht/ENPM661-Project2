# ENPM661-Project2

## Introduction

The goal of this project is to find the optimal path through a Cartesian maze with obstacles for both a point and rigid (circular) robot. The search algorithm that is used to find the path is Dijkstra, which is similar to Breadth First Search but with the addition of consideration to movement cost. For this project the robot is able to move in 8 directions: (N, NE, E, SE, S, SW, W, NW). The cardinal directions (N,E,S,W) have a cost of 1 and the sub-cardinal directions (NE,SE,SW,NW) have a cost of <img src="https://render.githubusercontent.com/render/math?math=\sqrt{2}"> . There are two mazes included in the repository.

Maze 1, which has a grid size of 200x100 and two obstacles a square and a circle.

![maze1](https://github.com/jaybrecht/ENPM661-Project2/blob/classes/Images/maze1.png)

Maze 2 is more complicated with a larger grid size of 300x200 and 6 obstacles, a concave polygon, a circle, an ellipse, a rotated rectangle, and a diamond. 

![maze2](https://github.com/jaybrecht/ENPM661-Project2/blob/classes/Images/maze2.png)

The user specifies a start point and goal point in the maze and the program finds the optimal path to the goal.

The program generates a visualization of the searched nodes in yellow 

![searching_nodes](https://github.com/jaybrecht/ENPM661-Project2/blob/classes/Images/seraching_nodes.png)

And plots the path to the goal in red.

![solution](https://github.com/jaybrecht/ENPM661-Project2/blob/classes/Images/solution.png)

The code has two main modes point robot and rigid robot.

## Instructions for Running the Program

### Point Robot

To run the point robot code, clone this repository, open a new terminal window and type `python Dijkstra_point.py`, if you have older versions of python installed you may need to run `python3` instead. The program will prompt you to enter a start point and goal point. If the points are valid and a solution is possible, the program will then show the visualization of the solution. This mode treats the robot as a point so the robot can occupy any pixels in the grid. 

### Rigid Robot

To run the point robot code, clone this repository, open a new terminal window and type `python Dijkstra_point.py`, if you have older versions of python installed you may need to run `python3` instead. The program will prompt you to enter a start point and goal point and the radius of your robot. If the points are valid and a solution is possible, the program will then show the visualization of the solution. This mode treats your robot as a circle with the given radius and expands all of the obstacles by the radius.

![expanded_obstacles](https://github.com/jaybrecht/ENPM661-Project2/blob/classes/Images/expanded_obstacles.png)

It then solves the maze the same way as the point robot. 


## Creating New Mazes

The maze is generated by reading in a text file. New mazes can be generated by creating new text files that follow the same format. 

The first parameters needed to establish a maze is the height and width of the maze in points. This can be specified by the lines. **Note it is crucial to use a colon** `:`

`height: XX`
`width: XX`

The code currently supports four different types of obstacles; circles, ellipses, polygons, and rotated rectangles. Depending on the type of obstacle you want to generate, different parameters are needed. 

### Circle
For a circle the file needs to contain a center point (x,y) and a radius (r). You may also specify a color in BGR (B,G,R). Each value should be between 0 and 255. The lines in text file should look like this

    circle
        center: x,y
        radius: r
        color: B,G,R

### Ellipse
To add a rotated rectangle obstacle you need to specify the center point (x,y), the major and minor axis (a1,a2), the rotation angle in degrees, and the start and end angles in degrees. To specify the entire ellipse the start should be 0 and the end 360. You may also specify a color in BGR (B,G,R). The lines in text file should look like this

    ellipse
        center: x,y
        axis: a1,a2
        angle: a 
        start: a
        end: a
        color: B,G,R

### Polygon
To add a polygon obstacle you need to specify all points that make up the exterior of the polygon in the form (x,y). The program will draw lines between each point so the order matters. You may also specify a color in BGR (B,G,R). The lines in text file should look like this

    polygon
        point: x1,y1
        point: x2,y2
        point: x3,y3
        point: x4,y4
        color: B,G,R

### Rotated Rectangle
To add a rotated rectangle obstacle you need to specify the position of the starting corner (the code currently only supports the starting corner as the bottom right corner), the length of both sides l1 and l2, and the rotation angle. You may also specify a color in BGR (B,G,R). The lines in text file should look like this

    rotatedrect
        start_point: x,y
        l1: l1
        l2: l2
        angle: a
        color: B,G,R

## Dependencies 

    cv2
    math
    numpy
    collections
    time
    shapely