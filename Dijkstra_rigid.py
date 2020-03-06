import cv2 
from maze import Maze
from robot import RigidRobot

write_to_video = False
show_visualization = True
user_input = False
search_type = 'D' # D for Dijkstra, B for BFS

# Construct maze object
scale = 5
maze = Maze('maze2.txt',scale)

# Ask user for start point and goal point
if user_input:
    print('Please enter a start point (x,y)')
    start_str_x = input('start x: ')
    start_str_y = input('start y: ')
    start_point = (int(start_str_x),int(start_str_y))
else:
    start_point = (5,5)

# Check if start point is valid in maze 
if maze.in_maze(start_point):
    pass
else:
    print("The start point is not valid")
    exit()
    
if user_input:
    print('Please enter a goal point (x,y)')
    start_str_x = input('start x: ')
    start_str_y = input('start y: ')
    goal_point = (int(start_str_x),int(start_str_y))
else:
  goal_point = (295,195)

# Check if goal point is valid in maze 
if maze.in_maze(goal_point):
    pass
else:
    print("The goal point is not valid")
    exit()

if user_input:
    print('Please enter the size of your robot')
    size_str = input('radius: ')
    robot_size = int(size_str)
    print('Please enter the clearance for your robot')
    clear_str = input('clearance: ')
    clearance = int(clear_str)
else:
    robot_size = 5
    clearance = 3

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

# Construct the robot
robot = RigidRobot(maze,start_point,goal_point,robot_size,clearance)

# Expand obstacles
robot.maze.expand_obstacles(robot.radius+robot.clearance)

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
