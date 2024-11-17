# Collaborator : Jusung Kim
# ****************************************#

# Q1 Implement and visualize Dijkstra's algorithm
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
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index  # index of previous Node

    def planning(self, start_x, start_y, goal_x, goal_y):
        start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
                               self.calc_xy_index(start_y, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
                              self.calc_xy_index(goal_y, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            # Find the node in open_set with the least cost and assign it to current
            current = min(open_set.values(), key=lambda node: node.cost)

            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            # Check if the current Node is the goal node
            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from open set and add it to closed set
            del open_set[self.calc_grid_index(current)]
            closed_set[self.calc_grid_index(current)] = current

            # Expand the search to the neighboring nodes
            for i, _ in enumerate(self.motion):
                node_x = current.x + self.motion[i][0]
                node_y = current.y + self.motion[i][1]
                node_cost = current.cost + self.motion[i][2]
                new_node = self.Node(node_x, node_y, node_cost, self.calc_grid_index(current))

                if not self.verify_node(new_node):
                    continue

                if self.calc_grid_index(new_node) in closed_set:
                    continue

                if self.calc_grid_index(new_node) not in open_set:
                    open_set[self.calc_grid_index(new_node)] = new_node
                else:
                    if open_set[self.calc_grid_index(new_node)].cost > new_node.cost:
                        open_set[self.calc_grid_index(new_node)] = new_node

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
        return (node.x - self.min_x) / self.resolution, (node.y - self.min_y) / self.resolution

    def verify_node(self, node):
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x or px >= self.max_x or py < self.min_y or py >= self.max_y:
            return False

        if self.obstacle_map[node.x][node.y]:
            return False

        return True

    def calc_obstacle_map(self, obstacle_x, obstacle_y):
        self.min_x = round(min(obstacle_x))
        self.min_y = round(min(obstacle_y))
        self.max_x = round(max(obstacle_x))
        self.max_y = round(max(obstacle_y))

        self.x_width = round((self.max_x - self.min_x) / self.resolution)
        self.y_width = round((self.max_y - self.min_y) / self.resolution)

        # DO NOT ALTER THE NEXT TWO LINES.
        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        
        for ix in range(self.x_width):
            x = self.calc_grid_position(ix, self.min_x)
            for iy in range(self.y_width):
                y = self.calc_grid_position(iy, self.min_y)
                for obs_x, obs_y in zip(obstacle_x, obstacle_y):
                    d = math.sqrt((obs_x - x) ** 2 + (obs_y - y) ** 2)
                    if d <= self.robot_radius:
                        self.obstacle_map[ix][iy] = True

    @staticmethod
    def get_motion_model():
        # dx, dy, cost
        motion = [[1, 1, 0],
                  [0, 1, 0],
                  [-1, 0, 1],
                  [1, -1, 1],
                  [0, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [0, 1, math.sqrt(2)]]
        return motion

def main():
    print(__file__ + " start!!")
    
    # start and goal position
    start_x = 5.0  # [m]
    start_y = 5.0  # [m]
    goal_x = 50.0  # [m]
    goal_y = 50.0  # [m]
    cell_size = 2.0  # [m]
    robot_radius = 1.0  # [m]
    
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


# Q2 Implement and visualize A* algorithm

import math

import matplotlib.pyplot as plt

show_animation = True

class AStarPlanner:

    def __init__(self, obstacle_x, obstacle_y, resolution, robot_radius):

        self.resolution = resolution
        self.robot_radius = robot_radius
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.calc_obstacle_map(obstacle_x, obstacle_y)
        self.motion = self.get_motion_model()
        

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  # index of grid
            self.y = y  # index of grid
            self.cost = cost
            self.parent_index = parent_index

    def planning(self, start_x, start_y, goal_x, goal_y):

        start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
                               self.calc_xy_index(start_y, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
                              self.calc_xy_index(goal_y, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            current = None
            current_index = min(open_set, key=lambda idx: open_set[idx].cost + self.calc_heuristic(open_set[idx], goal_node))
            current = open_set[current_index]

            #Check if open_set is empty. If so, break out of the while loop
            #Find the node in open_set with least cost to come (g) + cost to go (heuristic) and assign it to current
            #DO NOT ALTER THE NEXT 8 LINES
            if show_animation: 
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                plt.gcf().canvas.mpl_connect('key_release_event',
                                             lambda event: [exit(
                                                 0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            # Check if current Node is equal to the goal. If so, assign goal_node.parent_index and goal_node.cost to the appropriate values and break out of the while loop
            # If current node isnt the goal, remove the current node from the open set and add it to the closed set
            # Use the motion model to expand the search to other neighbors in the grid.
            # Check if the neighbouring cell is a part of closed set. If so, move on.
            # If the neighboring cell is not within bounds of the state space, move on.
            # If the neighboring cell is neither in open_set or closed_set, add it to the open set. 
            # If the neighboring cell is a part of the open cell, will expanding from the current node reduce the total cost to reach the neighbor? If so, replace it with the current node. (Essentially changing its parent and cost).

            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break
            del open_set[current_index] 
            closed_set[current_index] = current

            for move_x, move_y, move_cost in self.motion:
                node_x = current.x + move_x
                node_y = current.y + move_y
                node_cost = current.cost + move_cost
                neighbor_node = self.Node(node_x, node_y, node_cost, current_index)
                neighbor_index = self.calc_grid_index(neighbor_node)

                if not self.verify_node(neighbor_node):
                    continue

                if neighbor_index in closed_set:
                    continue

                if neighbor_index not in open_set:
                    open_set[neighbor_index] = neighbor_node
                else:
                    if open_set[neighbor_index].cost > neighbor_node.cost:
                        open_set[neighbor_index] = neighbor_node


        rx, ry = self.calc_final_path(goal_node, closed_set)

        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        # generate final course
        rx, ry = [self.calc_grid_position(goal_node.x, self.min_x)], [
            self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index

        return rx, ry

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        pos = index * self.resolution + min_position
        return pos

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.x,node.y)

    def verify_node(self, node):
        if node.x < 0 or node.x >= self.x_width or node.y < 0 or node.y >= self.y_width:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False
        return True


    def calc_obstacle_map(self, obstacle_x, obstacle_y):#This fuction is used to populate the obstacle map of the state space. The value is true if a specific cell in the map overlaps with an obstacle and false if not.
        #Find the minimum and maximum bounds for x and y
        #Use the bounds to obtain the width along x and y axes and store them in self.x_width and self.y_width respectively
        #DO NOT ALTER THE NEXT TWO LINES.
        self.min_x = min(obstacle_x)
        self.min_y = min(obstacle_y)
        self.max_x = max(obstacle_x)
        self.max_y = max(obstacle_y)

        self.x_width = int((self.max_x - self.min_x) / self.resolution) + 1
        self.y_width = int((self.max_y - self.min_y) / self.resolution) +1

        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]

        for i in range(len(obstacle_x)):
            ox = self.calc_xy_index(obstacle_x[i], self.min_x)
            oy = self.calc_xy_index(obstacle_y[i], self.min_y)
            self.obstacle_map[ox][oy] = True
        #For each cell in self.obstacle_map, use the calculations above to assign it as boolean True if the cell overlaps with an obstacle and boolean False if it doesn't

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
    start_x = 5.0  # [m]
    start_y = 5.0  # [m]
    goal_x = 50.0  # [m]
    goal_y = 50.0  # [m]
    cell_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    #Feel free to change the obstacle positions and test the implementation on various scenarios
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

    if show_animation:  # pragma: no cover
        plt.plot(obstacle_x, obstacle_y, ".k")
        plt.plot(start_x, start_y, "og")
        plt.plot(goal_x, goal_y, "xb")
        plt.grid(True)
        plt.axis("equal")

    a_star = AStarPlanner(obstacle_x, obstacle_y, cell_size, robot_radius)
    rx, ry = a_star.planning(start_x, start_y, goal_x, goal_y)

    if show_animation:  # pragma: no cover
        plt.plot(rx, ry, "-r")
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()


# Q3 Change the environments for the A* and Dijkstra’s implementation such that the
# time taken for both the algorithms is the same. Use python’s time library to show that the
# time taken by both is approximately the same (±0.2 seconds) . (Note: The heuristic function
# must not be altered.)

# After several tries in modifying the environment, I was able to get a little closer in the time difference 
# between the two planning algorithms, but not exactly the one which is required

import heapq
import math
import time
import matplotlib.pyplot as plt

show_animation = True  # Set to True if you want to visualize the results

class AStarPlanner:

    def __init__(self, obstacle_x, obstacle_y, resolution, robot_radius):
        self.resolution = resolution
        self.robot_radius = robot_radius
        self.min_x, self.min_y = 0, 0
        self.max_x, self.max_y = 0, 0
        self.obstacle_map = None
        self.x_width, self.y_width = 0, 0
        self.calc_obstacle_map(obstacle_x, obstacle_y)
        self.motion = self.get_motion_model()

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x
            self.y = y
            self.cost = cost
            self.parent_index = parent_index
    # @staticmethod
    def planning(self, start_x, start_y, goal_x, goal_y):
        start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
                               self.calc_xy_index(start_y, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
                              self.calc_xy_index(goal_y, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while open_set:
            current_index = min(open_set, key=lambda idx: open_set[idx].cost + self.calc_heuristic(open_set[idx], goal_node))
            current = open_set[current_index]

            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[current_index]
            closed_set[current_index] = current

            for move_x, move_y, move_cost in self.motion:
                node_x = current.x + move_x
                node_y = current.y + move_y
                node_cost = current.cost + move_cost
                neighbor_node = self.Node(node_x, node_y, node_cost, current_index)
                neighbor_index = self.calc_grid_index(neighbor_node)

                if not self.verify_node(neighbor_node):
                    continue

                if neighbor_index in closed_set:
                    continue

                if neighbor_index not in open_set:
                    open_set[neighbor_index] = neighbor_node
                else:
                    if open_set[neighbor_index].cost > neighbor_node.cost:
                        open_set[neighbor_index] = neighbor_node

        return self.calc_final_path(goal_node, closed_set)

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

    @staticmethod
    def calc_heuristic(n1, n2):
        w = 1.0  # weight of heuristic
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        return (node.x, node.y)

    def verify_node(self, node):
        if node.x < 0 or node.x >= self.x_width or node.y < 0 or node.y >= self.y_width:
            return False
        if self.obstacle_map[node.x][node.y]:
            return False
        return True

    def calc_obstacle_map(self, obstacle_x, obstacle_y):
        self.min_x = min(obstacle_x)
        self.min_y = min(obstacle_y)
        self.max_x = max(obstacle_x)
        self.max_y = max(obstacle_y)
        self.x_width = int((self.max_x - self.min_x) / self.resolution) + 1
        self.y_width = int((self.max_y - self.min_y) / self.resolution) + 1
        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        for i in range(len(obstacle_x)):
            ox = self.calc_xy_index(obstacle_x[i], self.min_x)
            oy = self.calc_xy_index(obstacle_y[i], self.min_y)
            self.obstacle_map[ox][oy] = True

    @staticmethod
    def get_motion_model():
        motion = [[1, 0, 1], [0, 1, 1], [-1, 0, 1], [0, -1, 1],
                  [-1, -1, math.sqrt(2)], [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)], [1, 1, math.sqrt(2)]]
        return motion
class DijkstraPlanner(AStarPlanner):
    # @staticmethod
    def planning(self, start_x, start_y, goal_x, goal_y):
        start_node = self.Node(self.calc_xy_index(start_x, self.min_x),
                               self.calc_xy_index(start_y, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(goal_x, self.min_x),
                              self.calc_xy_index(goal_y, self.min_y), 0.0, -1)

        open_set, closed_set = dict(), dict()
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            # Find the node in open_set with the least cost and assign it to current
            current = min(open_set.values(), key=lambda node: node.cost)

            if show_animation:  # pragma: no cover
                plt.plot(self.calc_grid_position(current.x, self.min_x),
                         self.calc_grid_position(current.y, self.min_y), "xc")
                plt.gcf().canvas.mpl_connect(
                    'key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
                if len(closed_set.keys()) % 10 == 0:
                    plt.pause(0.001)

            # Check if the current Node is the goal node
            if current.x == goal_node.x and current.y == goal_node.y:
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            # Remove the item from open set and add it to closed set
            del open_set[self.calc_grid_index(current)]
            closed_set[self.calc_grid_index(current)] = current

            # Expand the search to the neighboring nodes
            for i, _ in enumerate(self.motion):
                node_x = current.x + self.motion[i][0]
                node_y = current.y + self.motion[i][1]
                node_cost = current.cost + self.motion[i][2]
                new_node = self.Node(node_x, node_y, node_cost, self.calc_grid_index(current))

                if not self.verify_node(new_node):
                    continue

                if self.calc_grid_index(new_node) in closed_set:
                    continue

                if self.calc_grid_index(new_node) not in open_set:
                    open_set[self.calc_grid_index(new_node)] = new_node
                else:
                    if open_set[self.calc_grid_index(new_node)].cost > new_node.cost:
                        open_set[self.calc_grid_index(new_node)] = new_node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry
    @staticmethod
    def calc_heuristic(n1, n2):
        return 0  # Dijkstra has no heuristic, so return 0
def main():
    print("A* and Dijkstra Comparison Start!")

    start_x, start_y = 5.0, 5.0
    goal_x, goal_y = 40.0, 40.0
    cell_size, robot_radius = 2.0, 1.0

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
    for i in range(0, 45):
        obstacle_x.append(10.0)
        obstacle_y.append(i)
    for i in range(10, 45):
        obstacle_x.append(i)
        obstacle_y.append(45)
    for i in range(0, 50):
        obstacle_x.append(i)
        obstacle_y.append(50)
    for i in range(40, 51):
        obstacle_x.append(50)
        obstacle_y.append(i)
    for i in range(10, 30):
        obstacle_x.append(40.0)
        obstacle_y.append(i)
    for i in range(30, 45):
        obstacle_x.append(35)
        obstacle_y.append(i)
    for i in range(30, 40):
        obstacle_x.append(45)
        obstacle_y.append(i)


    # for i in range(0, 60):
    #     obstacle_x.append(i)  
    #     obstacle_y.append(0.0)
    
    # for i in range(0, 60):
    #     obstacle_x.append(60.0)  
    #     obstacle_y.append(i)
    
    # for i in range(0, 61):
    #     obstacle_x.append(i)  
    #     obstacle_y.append(60.0)
    
    # for i in range(0, 61):
    #     obstacle_x.append(0.0)  
    #     obstacle_y.append(i)

    # for i in range(10, 50):
    #     obstacle_x.append(30.0)  
    #     obstacle_y.append(i)

    # for i in range(10, 20):
    #     obstacle_x.append(10.0)
    #     obstacle_y.append(i)
    
    # for i in range(10, 20):
    #     obstacle_x.append(15.0)
    #     obstacle_y.append(20 + i)

    # for i in range(10, 30):
    #     obstacle_x.append(40.0)
    #     obstacle_y.append(30 + i)

    if show_animation:
        plt.plot(obstacle_x, obstacle_y, ".k")
        plt.plot(start_x, start_y, "og")
        plt.plot(goal_x, goal_y, "xb")
        plt.grid(True)
        plt.axis("equal")

    # Time A* Planning
    rx, ry = [], []
    average_time_difference_runs = 5
    difference_total = 0
    for i in range(0,average_time_difference_runs):
        start_time = time.time()
        a_star = AStarPlanner(obstacle_x, obstacle_y, cell_size, robot_radius)
        a_star_rx, a_star_ry = a_star.planning(start_x, start_y, goal_x, goal_y)
        end_time = time.time()

        a_star_time = end_time - start_time

        print("********************************************************************")
        print("Time times taken for A* and Dijkstra's algorithm, run number: ", i)

        print("Time taken for A*'s algorithm: ", a_star_time)

        # time function
        start_time = time.time()
        dijkstra = DijkstraPlanner(obstacle_x, obstacle_y, cell_size, robot_radius)
        dijkstra_rx, dijkstra_ry = dijkstra.planning(start_x, start_y, goal_x, goal_y)
        end_time = time.time()

        dijkstra_time = end_time - start_time

        difference_total += abs(dijkstra_time - a_star_time)

        # assert error if the time difference is less than 0.2 seconds
        # assert abs(dijkstra_time - a_star_time) < 0.2

        print("Time taken for Dijkstra's algorithm: ", dijkstra_time)

        print("Time difference: ", abs(dijkstra_time - a_star_time))

        print("End of run number: ", i)
    print("Average time difference: ", difference_total/average_time_difference_runs)

    if show_animation:
        plt.plot(a_star_rx, a_star_ry, "-r", label="A* Path")
        plt.plot(dijkstra_rx, dijkstra_ry, "-b", label="Dijkstra Path")
        plt.legend()
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()


#Q4 Is it possible for A* to be slower than Dijkstra’s at finding a solution? If so, give an example.

# A* can explore many unnecessary nodes due to a poor heuristic.
# Yes, it is indeed possible for the A* algorithm to be slower than Dijkstra's algorithm under certain conditions. 
# If the heuristic used in A* is not effective (for instance, it could overestimate the cost or provide poor guidance), A* might take longer to find a solution compared to Dijkstra's algorithm.

import heapq
import math
import time
import random

class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index

class AStarPlanner:
    def __init__(self, heuristic=lambda x, y: 0):
        self.heuristic = heuristic

    def planning(self, start, goal):
        open_set = {}
        closed_set = {}
        start_node = Node(start[0], start[1], 0.0, -1)
        goal_node = Node(goal[0], goal[1], 0.0, -1)
        open_set[(start[0], start[1])] = start_node

        while open_set:
            current_index = min(open_set, key=lambda idx: open_set[idx].cost + self.heuristic(*idx))
            current = open_set[current_index]

            if (current.x, current.y) == (goal_node.x, goal_node.y):
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[current_index]
            closed_set[current_index] = current

            # Explore neighbors
            for move_x, move_y, move_cost in [(1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1)]:
                neighbor_node = Node(current.x + move_x, current.y + move_y,
                                     current.cost + move_cost, current_index)

                if (neighbor_node.x, neighbor_node.y) in closed_set:
                    continue

                if (neighbor_node.x, neighbor_node.y) not in open_set:
                    open_set[(neighbor_node.x, neighbor_node.y)] = neighbor_node
                elif open_set[(neighbor_node.x, neighbor_node.y)].cost > neighbor_node.cost:
                    open_set[(neighbor_node.x, neighbor_node.y)] = neighbor_node

        return self.reconstruct_path(goal_node, closed_set)

    def reconstruct_path(self, goal_node, closed_set):
        path = []
        parent_index = goal_node.parent_index
        while parent_index != -1:
            path.append((goal_node.x, goal_node.y))
            node = closed_set[parent_index]
            parent_index = node.parent_index
            goal_node = node
        return path[::-1]

class DijkstraPlanner:
    def planning(self, start, goal):
        open_set = {}
        closed_set = {}
        start_node = Node(start[0], start[1], 0.0, -1)
        open_set[(start[0], start[1])] = start_node

        while open_set:
            current_index = min(open_set, key=lambda idx: open_set[idx].cost)
            current = open_set[current_index]

            if (current.x, current.y) == (goal[0], goal[1]):
                break

            del open_set[current_index]
            closed_set[current_index] = current

            # Explore neighbors
            for move_x, move_y, move_cost in [(1, 0, 1), (0, 1, 1), (-1, 0, 1), (0, -1, 1)]:
                neighbor_node = Node(current.x + move_x, current.y + move_y,
                                     current.cost + move_cost, current_index)

                if (neighbor_node.x, neighbor_node.y) in closed_set:
                    continue

                if (neighbor_node.x, neighbor_node.y) not in open_set:
                    open_set[(neighbor_node.x, neighbor_node.y)] = neighbor_node
                elif open_set[(neighbor_node.x, neighbor_node.y)].cost > neighbor_node.cost:
                    open_set[(neighbor_node.x, neighbor_node.y)] = neighbor_node

        return self.reconstruct_path(current, closed_set)

    def reconstruct_path(self, current, closed_set):
        path = []
        while current:
            path.append((current.x, current.y))
            current = closed_set.get((current.x, current.y), None)
        return path[::-1]

def main():
    start = (0, 0)
    goal = (10, 10)

    # poor heuristic (zero heuristic)
    heuristic = lambda x, y: 0  

    # Run multiple times to measure time
    num_runs = 100
    a_star_total_time = 0
    dijkstra_total_time = 0

    for _ in range(num_runs):
        # Timing A*
        a_star_planner = AStarPlanner(heuristic)
        start_time = time.time()
        a_star_path = a_star_planner.planning(start, goal)
        a_star_time = time.time() - start_time
        a_star_total_time += a_star_time

        # Timing Dijkstra
        dijkstra_planner = DijkstraPlanner()
        start_time = time.time()
        dijkstra_path = dijkstra_planner.planning(start, goal)
        dijkstra_time = time.time() - start_time
        dijkstra_total_time += dijkstra_time

    print("Average A* Time:", a_star_total_time / num_runs)
    print("Average Dijkstra Time:", dijkstra_total_time / num_runs)

if __name__ == '__main__':
    main()

# Q5 In probabilistic road map.py, the code for PRM generation is already completed.
# Using the PRM, A template has been provided to use Dijkstra’s on the existing PRM to find
# the solution. Finish the implementation such that a similar output is obtained (Since PRM
# is probabilistic, the exact output isn’t expected. However, the path between the start and
# goal must be found):

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# Parameters
N_SAMPLE = 500  # Number of sample points
N_KNN = 10  # Number of edges from one sampled point
MAX_EDGE_LEN = 30.0  # Maximum edge length
show_animation = True


class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index


def prm_planning(start_x, start_y, goal_x, goal_y,
                 obstacle_x_list, obstacle_y_list, robot_radius, *, rng=None):
    
    # Create a KDTree for obstacle points for efficient querying
    obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)

    # Sample points in the configuration space
    sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
                                       robot_radius, obstacle_x_list, obstacle_y_list,
                                       obstacle_kd_tree, rng)

    if show_animation:
        plt.plot(sample_x, sample_y, ".b", label="Sampled Points")

    # Generate the road map based on sampled points
    road_map = generate_road_map(sample_x, sample_y, robot_radius, obstacle_kd_tree)

    # Use Dijkstra's algorithm to find the path from start to goal
    rx, ry = dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)

    return rx, ry

def is_collision(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_kd_tree):
    x = start_x
    y = start_y
    dx = goal_x - start_x
    dy = goal_y - start_y
    yaw = math.atan2(dy, dx)
    d = math.hypot(dx, dy)

    if d >= MAX_EDGE_LEN:
        return True  # Too far, cannot connect

    D = robot_radius
    n_step = round(d / D)

    for _ in range(n_step):
        dist, _ = obstacle_kd_tree.query([x, y])
        if dist <= robot_radius:
            return True  # Collision detected
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    dist, _ = obstacle_kd_tree.query([goal_x, goal_y])
    if dist <= robot_radius:
        return True  # Collision detected

    return False  # No collision


def generate_road_map(sample_x, sample_y, robot_radius, obstacle_kd_tree):
    # Generate the roadmap using KDTree for nearest neighbor search
    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):
        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]

            if not is_collision(ix, iy, nx, ny, robot_radius, obstacle_kd_tree):
                edge_id.append(indexes[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    return road_map


def dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y):
    # Find the shortest path using Dijkstra's algorithm
    start_node = Node(start_x, start_y, 0.0, -1)
    goal_node = Node(goal_x, goal_y, 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    while True:
        # Find the Node in open_set with least cost and assign it to current
        current = min(open_set.values(), key=lambda node: node.cost)  # Node with least cost
        current_index = [i for i, node in open_set.items() if node == current][0]

        if show_animation and len(closed_set.keys()) % 2 == 0:
            plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        # Check if the current Node is the goal node
        if math.isclose(current.x, goal_x, abs_tol=1e-5) and math.isclose(current.y, goal_y, abs_tol=1e-5):
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        # Remove the current node from open set and add it to closed set
        open_set.pop(current_index)
        closed_set[current_index] = current

        # Expand the search to neighboring nodes
        for neighbor_index in road_map[current_index]:
            if neighbor_index in closed_set:
                continue  # Skip if it's already evaluated

            neighbor_node = Node(sample_x[neighbor_index], sample_y[neighbor_index], 0.0, current_index)
            new_cost = current.cost + math.hypot(neighbor_node.x - current.x, neighbor_node.y - current.y)

            if neighbor_index in open_set:
                if new_cost < open_set[neighbor_index].cost:
                    open_set[neighbor_index].cost = new_cost
                    open_set[neighbor_index].parent_index = current_index
            else:
                neighbor_node.cost = new_cost
                open_set[neighbor_index] = neighbor_node

    # Backtrack to find the path
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry


def sample_points(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_x, obstacle_y, obstacle_kd_tree, rng):
    # Sample points in the environment, avoiding obstacles
    max_x = max(obstacle_x)
    max_y = max(obstacle_y)
    min_x = min(obstacle_x)
    min_y = min(obstacle_y)

    sample_x, sample_y = [], []

    if rng is None:
        rng = np.random.default_rng()

    while len(sample_x) < N_SAMPLE:
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max_y - min_y)) + min_y

        dist, _ = obstacle_kd_tree.query([tx, ty])

        if dist >= robot_radius:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(start_x)
    sample_y.append(start_y)
    sample_x.append(goal_x)
    sample_y.append(goal_y)

    return sample_x, sample_y


def main(rng=None):
    print("PRM Planning start!")

    # Start and goal positions
    start_x = 10.0  # [m]
    start_y = 10.0  # [m]
    goal_x = 50.0  # [m]
    goal_y = 50.0  # [m]
    robot_radius = 5.0  # [m]

    obstacle_x = []
    obstacle_y = []

    # Define the obstacles
    for i in range(60):
        obstacle_x.append(i)
        obstacle_y.append(0.0)
    for i in range(60):
        obstacle_x.append(60.0)
        obstacle_y.append(i)
    for i in range(61):
        obstacle_x.append(i)
        obstacle_y.append(60.0)
    for i in range(61):
        obstacle_x.append(0.0)
        obstacle_y.append(i)
    for i in range(40):
        obstacle_x.append(20.0)
        obstacle_y.append(i)
    for i in range(40):
        obstacle_x.append(40.0)
        obstacle_y.append(60.0 - i)

    if show_animation:
        plt.plot(obstacle_x, obstacle_y, ".k", label="Obstacles")  # Plot obstacles
        plt.plot(start_x, start_y, "^r", label="Start")  # Plot start point
        plt.plot(goal_x, goal_y, "^c", label="Goal")  # Plot goal point
        plt.grid(True)
        plt.axis("equal")
        plt.legend()

    # Plan the path using PRM
    rx, ry = prm_planning(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, robot_radius, rng=rng)

    assert rx, 'Cannot find path'

    if show_animation:
        plt.plot(rx, ry, "-r", label="Path")  # Plot the path found
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()


# Q6  What is the average cost of the solution over 5 trials for the above implementation.
# What simple parameter change in PRM would result in the reduction of the average cost of
# solution provided by Dijkstra’s? Demonstrate by changing the appropriate parameter in the code.

# Average Cost over 5 trials: 125.26 for N_SAMPLE = 500
# Average Cost over 5 trials: 121.27 for N_SAMPLE = 1000

import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree

# Parameters
N_SAMPLE = 500  # Number of sample points
N_KNN = 10  # Number of edges from one sampled point
MAX_EDGE_LEN = 30.0  # Maximum edge length
show_animation = False


class Node:
    def __init__(self, x, y, cost, parent_index):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent_index = parent_index


def prm_planning(start_x, start_y, goal_x, goal_y,
                 obstacle_x_list, obstacle_y_list, robot_radius, *, rng=None):
    
    # Create a KDTree for obstacle points for efficient querying
    obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)

    # Sample points in the configuration space
    sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
                                       robot_radius, obstacle_x_list, obstacle_y_list,
                                       obstacle_kd_tree, rng)

    if show_animation:
        plt.plot(sample_x, sample_y, ".b", label="Sampled Points")

    # Generate the road map based on sampled points
    road_map = generate_road_map(sample_x, sample_y, robot_radius, obstacle_kd_tree)

    # Use Dijkstra's algorithm to find the path from start to goal
    rx, ry = dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)

    return rx, ry

def is_collision(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_kd_tree):
    """Check if a straight line path from (start_x, start_y) to (goal_x, goal_y) collides with obstacles."""
    x = start_x
    y = start_y
    dx = goal_x - start_x
    dy = goal_y - start_y
    yaw = math.atan2(dy, dx)
    d = math.hypot(dx, dy)

    if d >= MAX_EDGE_LEN:
        return True  # Too far, cannot connect

    D = robot_radius
    n_step = round(d / D)

    for _ in range(n_step):
        dist, _ = obstacle_kd_tree.query([x, y])
        if dist <= robot_radius:
            return True  # Collision detected
        x += D * math.cos(yaw)
        y += D * math.sin(yaw)

    dist, _ = obstacle_kd_tree.query([goal_x, goal_y])
    if dist <= robot_radius:
        return True  # Collision detected

    return False  # No collision


def generate_road_map(sample_x, sample_y, robot_radius, obstacle_kd_tree):
    # Generate the roadmap using KDTree for nearest neighbor search
    road_map = []
    n_sample = len(sample_x)
    sample_kd_tree = KDTree(np.vstack((sample_x, sample_y)).T)

    for (i, ix, iy) in zip(range(n_sample), sample_x, sample_y):
        dists, indexes = sample_kd_tree.query([ix, iy], k=n_sample)
        edge_id = []

        for ii in range(1, len(indexes)):
            nx = sample_x[indexes[ii]]
            ny = sample_y[indexes[ii]]

            if not is_collision(ix, iy, nx, ny, robot_radius, obstacle_kd_tree):
                edge_id.append(indexes[ii])

            if len(edge_id) >= N_KNN:
                break

        road_map.append(edge_id)

    return road_map


def dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y):
    # Find the shortest path using Dijkstra's algorithm
    start_node = Node(start_x, start_y, 0.0, -1)
    goal_node = Node(goal_x, goal_y, 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    while True:
        # Find the Node in open_set with least cost and assign it to current
        current = min(open_set.values(), key=lambda node: node.cost)  # Node with least cost
        current_index = [i for i, node in open_set.items() if node == current][0]

        if show_animation and len(closed_set.keys()) % 2 == 0:
            plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(current.x, current.y, "xg")
            plt.pause(0.001)

        # Check if the current Node is the goal node
        if math.isclose(current.x, goal_x, abs_tol=1e-5) and math.isclose(current.y, goal_y, abs_tol=1e-5):
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        # Remove the current node from open set and add it to closed set
        open_set.pop(current_index)
        closed_set[current_index] = current

        # Expand the search to neighboring nodes
        for neighbor_index in road_map[current_index]:
            if neighbor_index in closed_set:
                continue  # Skip if it's already evaluated

            neighbor_node = Node(sample_x[neighbor_index], sample_y[neighbor_index], 0.0, current_index)
            new_cost = current.cost + math.hypot(neighbor_node.x - current.x, neighbor_node.y - current.y)

            if neighbor_index in open_set:
                if new_cost < open_set[neighbor_index].cost:
                    open_set[neighbor_index].cost = new_cost
                    open_set[neighbor_index].parent_index = current_index
            else:
                neighbor_node.cost = new_cost
                open_set[neighbor_index] = neighbor_node

    # Backtrack to find the path
    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry


def sample_points(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_x, obstacle_y, obstacle_kd_tree, rng):
    # Sample points in the environment, avoiding obstacles
    max_x = max(obstacle_x)
    max_y = max(obstacle_y)
    min_x = min(obstacle_x)
    min_y = min(obstacle_y)

    sample_x, sample_y = [], []

    if rng is None:
        rng = np.random.default_rng()

    while len(sample_x) < N_SAMPLE:
        tx = (rng.random() * (max_x - min_x)) + min_x
        ty = (rng.random() * (max_y - min_y)) + min_y

        dist, _ = obstacle_kd_tree.query([tx, ty])

        if dist >= robot_radius:
            sample_x.append(tx)
            sample_y.append(ty)

    sample_x.append(start_x)
    sample_y.append(start_y)
    sample_x.append(goal_x)
    sample_y.append(goal_y)

    return sample_x, sample_y

def calculate_average_cost(trials=5, rng=None):
    """Calculate the average cost of the path found by Dijkstra's over multiple trials."""
    total_cost = 0.0

    for _ in range(trials):
        # Plan the path using PRM for each trial
        rx, ry = prm_planning(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, robot_radius, rng=rng)
        # Calculate the total cost of the path
        trial_cost = sum(math.hypot(rx[i+1] - rx[i], ry[i+1] - ry[i]) for i in range(len(rx) - 1))
        total_cost += trial_cost

    average_cost = total_cost / trials
    # print(f"Average Cost over {trials} trials: {average_cost:.2f}")
    return average_cost


def main(rng=None):
    print("PRM Planning start!")

    # Start and goal positions
    global start_x, start_y, goal_x, goal_y, robot_radius, obstacle_x, obstacle_y
    start_x = 10.0  # [m]
    start_y = 10.0  # [m]
    goal_x = 50.0  # [m]
    goal_y = 50.0  # [m]
    robot_radius = 5.0  # [m]

    obstacle_x = []
    obstacle_y = []

    # Define the obstacles
    for i in range(60):
        obstacle_x.append(i)
        obstacle_y.append(0.0)
    for i in range(60):
        obstacle_x.append(60.0)
        obstacle_y.append(i)
    for i in range(61):
        obstacle_x.append(i)
        obstacle_y.append(60.0)
    for i in range(61):
        obstacle_x.append(0.0)
        obstacle_y.append(i)
    for i in range(40):
        obstacle_x.append(20.0)
        obstacle_y.append(i)
    for i in range(40):
        obstacle_x.append(40.0)
        obstacle_y.append(60.0 - i)

    if show_animation:
        plt.plot(obstacle_x, obstacle_y, ".k", label="Obstacles")  # Plot obstacles
        plt.plot(start_x, start_y, "^r", label="Start")  # Plot start point
        plt.plot(goal_x, goal_y, "^c", label="Goal")  # Plot goal point
        plt.grid(True)
        plt.axis("equal")
        plt.legend()

    # Plan the path using PRM and calculate the average cost with original parameters
    original_avg_cost = calculate_average_cost(trials=5)
    print(f"Cost when N_SAMPLE = 500 is {original_avg_cost:.2f}")

    global N_SAMPLE
    N_SAMPLE = 1000
    new_avg_cost = calculate_average_cost(trials=5)
    print(f"Cost over N_SAMPLE = 1000 is {new_avg_cost:.2f}")


    # Plan the path using PRM
    rx, ry = prm_planning(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, robot_radius, rng=rng)

    assert rx, 'Cannot find path'

    if show_animation:
        plt.plot(rx, ry, "-r", label="Path")  # Plot the path found
        plt.pause(0.001)
        plt.show()

if __name__ == '__main__':
    main()

# Q7 A template has been provided to implement and visualize RRT algorithm via rrt.py.
# A brief psuedocode has been commented in the python file to help in the implementation.
# Finish the implementation such that a similar output is obtained (Since RRT is probabilistic,
# the exact output isn’t expected. However, the path between the start and goal must be
# found)

import math
import random
import matplotlib.pyplot as plt
import numpy as np

show_animation = True

class RRT:
    
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3.0,
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 ):
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

    def planning(self, animation=True):
        self.node_list = [self.start]  # Initialize the list with the start node
        
        for i in range(self.max_iter):
            # Sample a random node
            rnd_node = self.get_random_node()
            
            # Find the nearest node in the current tree
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_ind]
            
            # Steer towards the random node
            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)
            
            # If the new node is valid (inside the play area and not colliding)
            if self.check_if_outside_play_area(new_node) and not self.collision_check(new_node):
                self.node_list.append(new_node)
            
            # Animate the graph if needed
            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)
            
            # Check if we are close enough to the goal
            dist_to_goal, _ = self.calc_distance_and_angle(new_node, self.end)
            
            # If the new node is within range of the goal, steer towards the goal
            if dist_to_goal <= self.expand_dis:
                final_node = self.steer(new_node, self.end)
                
                # Check if we can reach the goal without collision
                if not self.collision_check(final_node):
                    final_node.x = self.end.x  # Snap the final node to the exact goal coordinates
                    final_node.y = self.end.y
                    self.node_list.append(final_node)
                    
                    # Return the path from the start to the goal
                    return self.generate_final_course(len(self.node_list) - 1)
            
            # Periodic animation during planning
            if animation and i % 5:
                self.draw_graph(rnd_node)
        
        return None  # Return None if the goal wasn't reached within max iterations
    def check_if_outside_play_area(self, node):
        if self.play_area is None:
            return True  # If there is no play area defined, assume it's valid

        if (self.play_area.xmin <= node.x <= self.play_area.xmax and
                self.play_area.ymin <= node.y <= self.play_area.ymax):
            return True  # Node is inside the play area
        return False  # Node is outside the play area

    def steer(self, from_node, to_node, extend_length=float("inf")):
        
        new_node = self.Node(from_node.x, from_node.y)
        distance, theta = self.calc_distance_and_angle(new_node, to_node)
        extend_length = min(extend_length, distance)
        
        n_expand = math.floor(extend_length / self.path_resolution)

        new_node.path_x = [new_node.x]  # Initialize with the start position
        new_node.path_y = [new_node.y]  # Initialize with the start position
        
        for i in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
        
        distance, _ = self.calc_distance_and_angle(new_node, to_node)
        if distance <= self.path_resolution:
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
        
        new_node.parent = from_node
        return new_node


    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        
        if rnd is not None:
            # Ensure rnd is not None before trying to access its attributes
            if rnd.x is not None and rnd.y is not None:
                plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        
        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)
        
        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                    self.play_area.xmax, self.play_area.xmin,
                    self.play_area.xmin],
                    [self.play_area.ymin, self.play_area.ymin,
                    self.play_area.ymax, self.play_area.ymax,
                    self.play_area.ymin],
                    "-k")
        
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)


    @staticmethod
    def plot_circle(x, y, size, color="-b"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return distance, theta

    def collision_check(self, node):
        if not node.path_x or not node.path_y:
            return False  # No path to check for collision

        for (ox, oy, size) in self.obstacle_list:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for dx, dy in zip(dx_list, dy_list)]
            
            if d_list and min(d_list) <= (size + self.robot_radius) ** 2:
                return True  # Collision

        return False  # Safe


    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand))
        else:
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
        min_index = dlist.index(min(dlist))
        return min_index

    def is_inside_play_area(self, node):
        if self.play_area is None:
            return True  # No play area, always True

        if (self.play_area.xmin <= node.x <= self.play_area.xmax and
                self.play_area.ymin <= node.y <= self.play_area.ymax):
            return True
        return False

    def generate_final_course(self, goal_ind):
        path = []
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent
        path.append([self.start.x, self.start.y])
        return path

def main(goal_x=6.0, goal_y=10.0):
    print("start " + __file__)
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2), (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    
    rrt = RRT(
        start=[0, 0],
        goal=[goal_x, goal_y],
        rand_area=[-2, 15],
        obstacle_list=obstacleList,
        robot_radius=0.8
    )
    
    path = rrt.planning(animation=show_animation)
    
    if path is None:
        print("Cannot find path")
    else:
        print("Found path!!")
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)
            plt.show()

if __name__ == '__main__':
    main()


# Q8 Does the above implementation work if the value of expand dis is changed from 3.0
# to 0.1? Why or why not? If not, what parameter can be changed to ensure the implementation
# works with expand dis being 0.1?

# Changing the expand_dis from 3 to 0.1 -> the expandable distance from one node to another in each successive iteration would be minimum
# I have changed the following factors:
# path_resoluton = 2 (I have tried out with path resolution 1 but I couldn't achieve reaching the goal. The smaller the path_resolution, the smaller would be the movement between nodes)
# goal_sample_rate = 10 (probability of sampling the goal will be increased and goal will be sampled as the target point instead of a random point in the search space)
# max_iter = 10000 (Since the movement in each successive iteration is very less, max_iter required would be more to reach the goal. I have tried out with max_iter = 5000. But the program exited without finding a path to the goal)

import math
import random
import matplotlib.pyplot as plt
import numpy as np

show_animation = True

class RRT:
    
    class Node:
        def __init__(self, x, y):
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None

    class AreaBounds:
        def __init__(self, area):
            self.xmin = float(area[0])
            self.xmax = float(area[1])
            self.ymin = float(area[2])
            self.ymax = float(area[3])

    def __init__(self,
                 start,
                 goal,
                 obstacle_list,
                 rand_area,
                 expand_dis=3,  # distance extended would be minimum for each iteration
                 path_resolution=0.5,
                 goal_sample_rate=5,
                 max_iter=500,
                 play_area=None,
                 robot_radius=0.0,
                 ):
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])
        self.min_rand = rand_area[0]
        self.max_rand = rand_area[1]
        if play_area is not None:
            self.play_area = self.AreaBounds(play_area)
        else:
            self.play_area = None
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.robot_radius = robot_radius

    def planning(self, animation=True): # Tree expanded until the goal is reached or the maximum iteration uis completed.
        self.node_list = [self.start]  # Initialize 
        for i in range(self.max_iter):
            rnd_node = self.get_random_node()  # Getting a random node

            nearest_index = self.get_nearest_node_index(self.node_list, rnd_node)
            nearest_node = self.node_list[nearest_index]

            new_node = self.steer(nearest_node, rnd_node, self.expand_dis)  # Extending path towards rnd_node

            # If the new node is valid, add it to the list
            if not self.collision_check(new_node) and self.check_if_outside_play_area(new_node):
                self.node_list.append(new_node)

            # Check if we've reached the goal within the defined radius
            if self.calc_distance_and_angle(new_node, self.end)[0] <= self.expand_dis:
                final_node = self.steer(new_node, self.end, self.expand_dis)
                if not self.collision_check(final_node):
                    return self.generate_final_course(final_node)  # Return the path to the goal

            if animation and i % 5 == 0:
                self.draw_graph(rnd_node)
        return None  # Return None if no path is found

    def check_if_outside_play_area(self, node):
        if self.play_area is None:
            return True  # If there is no play area defined, assume it's valid

        if (self.play_area.xmin <= node.x <= self.play_area.xmax and
                self.play_area.ymin <= node.y <= self.play_area.ymax):
            return True  # Node is inside the play area
        return False  # Node is outside the play area

    def steer(self, from_node, to_node, extend_length=float("inf")):
        
        new_node = self.Node(from_node.x, from_node.y)
        distance, theta = self.calc_distance_and_angle(new_node, to_node)
        extend_length = min(extend_length, distance)
        
        n_expand = math.floor(extend_length / self.path_resolution)

        new_node.path_x = [new_node.x]  # Initialize with the start position
        new_node.path_y = [new_node.y]  # Initialize with the start position
        
        for i in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
        
        distance, _ = self.calc_distance_and_angle(new_node, to_node)
        if distance <= self.path_resolution:
            new_node.x = to_node.x
            new_node.y = to_node.y
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
        
        new_node.parent = from_node
        return new_node


    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        
        if rnd is not None:
            # Ensure rnd is not None before trying to access its attributes
            if rnd.x is not None and rnd.y is not None:
                plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")
        
        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)
        
        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                    self.play_area.xmax, self.play_area.xmin,
                    self.play_area.xmin],
                    [self.play_area.ymin, self.play_area.ymin,
                    self.play_area.ymax, self.play_area.ymax,
                    self.play_area.ymin],
                    "-k")
        
        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.end.x, self.end.y, "xr")
        plt.axis("equal")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)


    @staticmethod
    def plot_circle(x, y, size, color="-b"):
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    @staticmethod
    def calc_distance_and_angle(from_node, to_node):
        dx = to_node.x - from_node.x
        dy = to_node.y - from_node.y
        distance = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return distance, theta

    def collision_check(self, node):
        if not node.path_x or not node.path_y:
            return False  # No path to check for collision

        for (ox, oy, size) in self.obstacle_list:
            dx_list = [ox - x for x in node.path_x]
            dy_list = [oy - y for y in node.path_y]
            d_list = [dx * dx + dy * dy for dx, dy in zip(dx_list, dy_list)]
            
            if d_list and min(d_list) <= (size + self.robot_radius) ** 2:
                return True  # Collision

        return False  # Safe


    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd = self.Node(random.uniform(self.min_rand, self.max_rand),
                            random.uniform(self.min_rand, self.max_rand))
        else:
            rnd = self.Node(self.end.x, self.end.y)
        return rnd

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x - rnd_node.x) ** 2 + (node.y - rnd_node.y) ** 2 for node in node_list]
        min_index = dlist.index(min(dlist))
        return min_index

    def is_inside_play_area(self, node):
        if self.play_area is None:
            return True  # No play area, always True

        if (self.play_area.xmin <= node.x <= self.play_area.xmax and
                self.play_area.ymin <= node.y <= self.play_area.ymax):
            return True
        return False

    def generate_final_course(self, goal_node):
        path = []
        node = goal_node

        # Backtrack from the goal node to the start node
        while node is not None:
            path.append((node.x, node.y))  # Add the coordinates to the path
            node = node.parent  # Move to the parent node

        return path[::-1]  # Reverse the path to get it from start to goal


def main(goal_x=6.0, goal_y=10.0):
    print("start " + __file__)
    obstacleList = [(5, 5, 1), (3, 6, 2), (3, 8, 2), (3, 10, 2), (7, 5, 2), (9, 5, 2), (8, 10, 1)]  # [x, y, radius]
    
    rrt = RRT(
        start=[0, 0],
        goal=[goal_x, goal_y],
        rand_area=[-2, 15],
        expand_dis=0.1,
        path_resolution=0.1,
        obstacle_list=obstacleList,
        robot_radius=0.8,
        max_iter= 10000
    )
    
    path = rrt.planning(animation=show_animation)
    
    if path is None:
        print("Cannot find path")
    else:
        print("Found path!!")
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)
            plt.show()

if __name__ == '__main__':
    main()
