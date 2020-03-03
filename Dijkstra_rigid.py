import cv2 
from maze import Maze
from robot import RigidRobot

write_to_video = False
show_visualization = True
search_type = 'D' # D for Dijkstra, B for BFS

# Construct maze object
scale = 5
maze = Maze('maze2.txt',scale)

# Ask user for start point and goal point
start_point = (0,0)
goal_point = (200,150)
robot_size = 3

# Check if points are valid in maze 
if maze.in_maze(start_point):
    pass
else:
    print("The start point is not valid")
    exit()

if maze.in_maze(goal_point):
    pass
else:
    print("The goal point is not valid")
    exit()

# Contstruct the robot
robot = RigidRobot(maze,start_point,goal_point,robot_size)

# Expand obstacles
robot.maze.expand_obstacles(robot.radius)

# Run Search
if search_type == 'D':
    robot.Dijkstra()
if search_type == 'B':
    robot.BFS()

if robot.foundGoal:
    robot.generate_path()
else:
    print('The goal could not be found')
    exit()

# Visualize the path
robot.visualize(show_visualization,write_to_video,50)
