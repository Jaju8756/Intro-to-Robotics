import matplotlib.pyplot as plt
import math
show_animation = True
class Dijkstra:
def __init__(self, obstacle_x, obstacle_y, resolution, robot_radius):
    self.min_x = None
    self.min_y = None
    self.max_x = None
    self.max_y = None
    self.x_width = None
    self.y_width = None
    self.obstacle_map = None
    self.resolution = resolution
    self.robot_radius = robot_radius
    self.calc_obstacle_map(obstacle_x, obstacle_y)
    self.motion = self.get_motion_model()
class Node:
def __init__(self, x, y, cost, parent_index):
self.x = x # index of grid
self.y = y # index of grid
self.cost = cost
self.parent_index = parent_index # index of previous Node
def planning(self, start_x, start_y, goal_x, goal_y):
start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
self.calc_xy_index(start_y, self.min_y), 0.0, -1)
goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
self.calc_xy_index(goal_y, self.min_y), 0.0, -1)
open_set, closed_set = dict(), dict()
open_set[self.calc_grid_index(start_node)] = start_node
while True:
#Find the Node in open_set with least cost and assign it to current
current = None #Replace None with code to assign the least costly node
in open_set
#DO NOT ALTER THE NEXT 8 LINES.
if show_animation: # pragma: no cover
plt.plot(self.calc_grid_position(current.x, self.min_x),
self.calc_grid_position(current.y, self.min_y), "xc")
plt.gcf().canvas.mpl_connect(
'key_release_event',
lambda event: [exit(0) if event.key == 'escape' else None])
if len(closed_set.keys()) % 10 == 0:
plt.pause(0.001)
# Check if the current Node is the goal node. If so, assign appropriate
values to goal_node.parent_index and goal_node.cost and break from the while loop
# If the current node is not the goal, remove the item from the open
set and add it to the closed set
# Use the motion model to expand the search to other neighbors in the
grid.
# Check if the neighbouring cell is a part of closed set. If so, move
on.
# If the neighboring cell is not within bounds of the state space, move
on.
# If the neighboring cell is neither in open_set or closed_set, add it
to the open set.
# If the neighboring cell is a part of the open cell, will expanding
from the current node reduce the total cost to reach the neighbor? If so, replace
it with the current node. (Essentially changing its parent and cost).
rx, ry = self.calc_final_path(goal_node, closed_set)
return rx, ry
def calc_final_path(self, goal_node, closed_set):
rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
self.calc_grid_position(goal_node.y, self.min_y)]
parent_index = goal_node.parent_index
while parent_index != -1:
n = closed_set[parent_index]
rx.append(self.calc_grid_position(n.x, self.min_x))
ry.append(self.calc_grid_position(n.y, self.min_y))
parent_index = n.parent_index
return rx, ry
def calc_grid_position(self, index, min_position):
pos = index * self.resolution + min_position
return pos
def calc_xy_index(self, position, min_position):
return round((position - min_position) / self.resolution)
def calc_grid_index(self, node):
return (node.x,node.y)
def verify_node(self, node): #This function tries to verify if the node is
within state space bounds and is collision-free
#Verify if the node is within bounds and isn't colliding with an obstacle
#Return False if node is invalid. Otherwise, return True
return True
def calc_obstacle_map(self, obstacle_x, obstacle_y): #This fuction is used to
populate the obstacle map of the state space. The value is true if a specific cell
in the map overlaps with an obstacle and false if not.
#Find the minimum and maximum bounds for x and y
#Use the bounds to obtain the width along x and y axes and store them in
self.x_width and self.y_width respectively
#DO NOT ALTER THE NEXT TWO LINES.
self.obstacle_map = [[False for _ in range(self.y_width)]
for _ in range(self.x_width)]
#For each cell in self.obstacle_map, use the calculations above to assign
it as boolean True if the cell overlaps with an obstacle and boolean False if it
doesn't
@staticmethod
def get_motion_model():
# dx, dy, cost
motion = [[1, 0, 1],
[0, 1, 1],
[-1, 0, 1],
[0, -1, 1],
[-1, -1, math.sqrt(2)],
[-1, 1, math.sqrt(2)],
[1, -1, math.sqrt(2)],
[1, 1, math.sqrt(2)]]
return motion
def main():
print(__file__ + " start!!")
# start and goal position
start_x = 5.0 # [m]
start_y = 5.0 # [m]
goal_x = 50.0 # [m]
goal_y = 50.0 # [m]
cell_size = 2.0 # [m]
robot_radius = 1.0 # [m]
#Feel free to change the obstacle positions and test the implementation on
various scenarios
obstacle_x, obstacle_y = [], []
for i in range(0, 60):
obstacle_x.append(i)
obstacle_y.append(0.0)
for i in range(0, 60):
obstacle_x.append(60.0)
obstacle_y.append(i)
for i in range(0, 61):
obstacle_x.append(i)
obstacle_y.append(60.0)
for i in range(0, 61):
obstacle_x.append(0.0)
obstacle_y.append(i)
for i in range(0, 40):
obstacle_x.append(20.0)
obstacle_y.append(i)
for i in range(0, 40):
obstacle_x.append(40.0)
obstacle_y.append(60.0 - i)
if show_animation:
plt.plot(obstacle_x, obstacle_y, ".k")
plt.plot(start_x, start_y, "og")
plt.plot(goal_x, goal_y, "xb")
plt.grid(True)
plt.axis("equal")
dijkstra = Dijkstra(obstacle_x, obstacle_y, cell_size, robot_radius)
rx, ry = dijkstra.planning(start_x, start_y, goal_x, goal_y)
if show_animation:
plt.plot(rx, ry, "-r")
plt.pause(0.01)
plt.show()
if __name__ == '__main__':
main()