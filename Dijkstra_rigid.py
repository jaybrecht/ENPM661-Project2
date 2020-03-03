import cv2 
from maze import Maze
from robot import RigidRobot

write_to_video = True
show_visualization = False

# Construct maze object
scale = 5
maze = Maze('maze2.txt',scale)

# Ask user for start point and goal point
start_point = (0,0)
goal_point = (100,100)
robot_size = 5

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

# Run Dijkstra Search
robot.Dijkstra()
if robot.foundGoal:
    robot.generate_path()
else:
    print('The goal could not be found')
    exit()

# Visualize the path
robot.visualize(show_visualization,write_to_video,50)
