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
    
    obstacle_kd_tree = KDTree(np.vstack((obstacle_x_list, obstacle_y_list)).T)

    sample_x, sample_y = sample_points(start_x, start_y, goal_x, goal_y,
                                       robot_radius, obstacle_x_list, obstacle_y_list,
                                       obstacle_kd_tree, rng)

    if show_animation:
        plt.plot(sample_x, sample_y, ".b", label="Sampled Points")

    road_map = generate_road_map(sample_x, sample_y, robot_radius, obstacle_kd_tree)

    rx, ry, cost = dijkstra_planning(start_x, start_y, goal_x, goal_y, road_map, sample_x, sample_y)

    return rx, ry, cost


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
    start_node = Node(start_x, start_y, 0.0, -1)
    goal_node = Node(goal_x, goal_y, 0.0, -1)

    open_set, closed_set = dict(), dict()
    open_set[len(road_map) - 2] = start_node

    while True:
        current = min(open_set.values(), key=lambda node: node.cost)
        current_index = [i for i, node in open_set.items() if node == current][0]

        if math.isclose(current.x, goal_x, abs_tol=1e-5) and math.isclose(current.y, goal_y, abs_tol=1e-5):
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            break

        open_set.pop(current_index)
        closed_set[current_index] = current

        for neighbor_index in road_map[current_index]:
            if neighbor_index in closed_set:
                continue

            neighbor_node = Node(sample_x[neighbor_index], sample_y[neighbor_index], 0.0, current_index)
            new_cost = current.cost + math.hypot(neighbor_node.x - current.x, neighbor_node.y - current.y)

            if neighbor_index in open_set:
                if new_cost < open_set[neighbor_index].cost:
                    open_set[neighbor_index].cost = new_cost
                    open_set[neighbor_index].parent_index = current_index
            else:
                neighbor_node.cost = new_cost
                open_set[neighbor_index] = neighbor_node

    rx, ry = [goal_node.x], [goal_node.y]
    parent_index = goal_node.parent_index
    while parent_index != -1:
        n = closed_set[parent_index]
        rx.append(n.x)
        ry.append(n.y)
        parent_index = n.parent_index

    return rx, ry, goal_node.cost  # Return cost as well


def sample_points(start_x, start_y, goal_x, goal_y, robot_radius, obstacle_x, obstacle_y, obstacle_kd_tree, rng):
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

    total_cost = 0
    trials = 5

    for _ in range(trials):
        rx, ry, cost = prm_planning(start_x, start_y, goal_x, goal_y, obstacle_x, obstacle_y, robot_radius, rng=rng)
        total_cost += cost
        assert rx, 'Cannot find path'

        if show_animation:
            plt.plot(rx, ry, "-r")  # Plot the path found

    average_cost = total_cost / trials
    print(f"Average cost over {trials} trials: {average_cost}")

    if show_animation:
        plt.pause(0.001)
        plt.show()


if __name__ == '__main__':
    main()
