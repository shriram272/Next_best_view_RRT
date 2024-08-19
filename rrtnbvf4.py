# import numpy as np
# import matplotlib.pyplot as plt
# import random

# class treeNode():
#     def __init__(self, locationX, locationY, gain=0.0):
#         self.locationX = locationX
#         self.locationY = locationY
#         self.children = []
#         self.parent = None
#         self.gain = gain

# class RRT_NBV():
#     def __init__(self, start, grid, size, theta_max, N, d_max, d_ideal, λ=0.1, Nmax=1000, NT_OL=1500, threshold=0.01):
#         self.root = treeNode(start[0], start[1])
#         self.grid = grid
#         self.size = size
#         self.theta_max = theta_max
#         self.N = N
#         self.d_max = d_max
#         self.d_ideal = d_ideal
#         self.λ = λ
#         self.Nmax = Nmax
#         self.NT_OL = NT_OL
#         self.threshold = threshold
#         self.coverage = np.zeros_like(grid)
#         self.tree = [self.root]
#         self.best_node = self.root
#         self.best_gain = 0
#         self.nearest_node = None
#         self.nearest_dist = float('inf')

#     def sample(self):
#         x = random.randint(0, self.grid.shape[1] - 1)
#         y = random.randint(0, self.grid.shape[0] - 1)
#         return np.array([x, y])

#     def unitVector(self, locStart, locEnd):
#         v = np.array([locEnd[0] - locStart.locationX, locEnd[1] - locStart.locationY])
#         return v / np.linalg.norm(v)

#     def add_child(self, parent, locationX, locationY, gain):
#         child_node = treeNode(locationX, locationY, gain)
#         parent.children.append(child_node)
#         child_node.parent = parent
#         self.tree.append(child_node)
#         self.updateCoverage(child_node)

#     def goto(self, locStart, locEnd):
#         move = self.size * self.unitVector(locStart, locEnd)
#         point = np.array([locStart.locationX + move[0], locStart.locationY + move[1]])
#         point[0] = min(max(0, point[0]), self.grid.shape[1] - 1)
#         point[1] = min(max(0, point[1]), self.grid.shape[0] - 1)
#         return point

#     def Nearest(self, root, point):
#         if not root:
#             return
#         dist = self.distance(root, point)
#         if dist < self.nearest_dist:
#             self.nearest_node = root
#             self.nearest_dist = dist
#         for child in root.children:
#             self.Nearest(child, point)

#     def inObst(self, locStart, locEnd):
#         unit = self.unitVector(locStart, locEnd)
#         testPoint = np.array([0.0, 0.0])
#         for i in range(int(np.linalg.norm([locEnd[0] - locStart.locationX, locEnd[1] - locStart.locationY]))):
#             testPoint[0] = min(self.grid.shape[1] - 1, locStart.locationX + i * unit[0])
#             testPoint[1] = min(self.grid.shape[0] - 1, locStart.locationY + i * unit[1])
#             if self.grid[round(testPoint[1].astype(int)), round(testPoint[0].astype(int))] == 1:
#                 return True
#         return False

#     def distance(self, node1, point):
#         return np.sqrt((node1.locationX - point[0]) ** 2 + (node1.locationY - point[1]) ** 2)

#     def resetNearestValues(self):
#         self.nearest_node = None
#         self.nearest_dist = float('inf')

#     def information_gain(self, x, y, theta, parent_gain=0, parent_path_cost=0):
#         total_gain = 0
#         rays = self.generate_rays(x, y, theta)
#         for angle, start_x, start_y in rays:
#             hit = self.trace_ray(start_x, start_y, angle)
#             if hit is not None:
#                 px, py = hit
#                 distance = np.sqrt((px - start_x) ** 2 + (py - start_y) ** 2)
#                 weight = max(0, (self.d_ideal - distance) ** 2)
#                 cell_value = self.grid[int(py), int(px)]
#                 uncertainty = 1 - abs(cell_value - 0.5) * 2
#                 total_gain += weight * uncertainty

#         path_cost = self.distance(self.nearest_node, [x, y])
#         penalized_gain = parent_gain + total_gain * np.exp(-self.λ * (parent_path_cost + path_cost))
#         return penalized_gain + 1e-5, path_cost

#     def generate_rays(self, x, y, theta):
#         rays = []
#         for i in range(self.N):
#             angle = theta + (-self.theta_max + (2 * i / (self.N - 1)) * self.theta_max)
#             rays.append((angle, x, y))
#         return rays

#     def trace_ray(self, x, y, angle):
#         dx = np.cos(angle)
#         dy = np.sin(angle)
#         for distance in np.arange(0, self.d_max, 1):
#             px = x + distance * dx
#             py = y + distance * dy
#             if 0 <= px < self.grid.shape[1] and 0 <= py < self.grid.shape[0]:
#                 cell_value = self.grid[int(py), int(px)]
#                 if cell_value == 1:
#                     return None
#                 elif 0 < cell_value < 1:
#                     return (px, py)
#             else:
#                 break
#         return None

#     def updateCoverage(self, node):
#         x, y = int(node.locationX), int(node.locationY)
#         for i in range(-self.size, self.size + 1):
#             for j in range(-self.size, self.size + 1):
#                 if 0 <= x + i < self.grid.shape[1] and 0 <= y + j < self.grid.shape[0]:
#                     self.coverage[y + j, x + i] = 1

#     def goalReached(self):
#         return np.sum(self.coverage >= self.threshold) / np.prod(self.grid.shape) > 0.9

#     def extract_best_path(self):
#         path = []
#         current_node = self.best_node
#         while current_node is not None:
#             path.append((current_node.locationX, current_node.locationY))
#             current_node = current_node.parent
#         path.reverse()
#         return path

#     def reset_tree(self):
#         self.tree = [self.root]
#         self.best_node = self.root
#         self.best_gain = 0

#     def continue_tree_construction(self):
#         return len(self.tree) < self.NT_OL and self.best_gain == 0

# # Set up the grid and the start position
# grid = np.load('obstacle_grid8.npy')  # Load your occupancy grid here
# start = np.array([100.0, 100.0])  # Starting position of the robot
# size = 30
# theta_max = np.pi / 4
# N = 10
# d_max = 50
# d_ideal = 30
# λ = 0.1
# Nmax = 1000
# NT_OL = 1500

# # Set up the plot
# fig = plt.figure("RRT-NBV Algorithm")
# plt.imshow(grid, cmap='binary')
# plt.plot(start[0], start[1], 'ro')
# plt.xlabel('X-axis $(m)$')
# plt.ylabel('Y-axis $(m)$')

# rrt_nbv = RRT_NBV(start, grid, size, theta_max, N, d_max, d_ideal, λ, Nmax, NT_OL)
# plt.pause(2)

# for i in range(rrt_nbv.NT_OL):
#     rrt_nbv.resetNearestValues()
#     print("Iteration:", i)

#     point = rrt_nbv.sample()
#     rrt_nbv.Nearest(rrt_nbv.root, point)

#     newX, newY = rrt_nbv.goto(rrt_nbv.nearest_node, point)

#     if not rrt_nbv.inObst(rrt_nbv.nearest_node, [newX, newY]):
#         theta = np.arctan2(newY - rrt_nbv.nearest_node.locationY, newX - rrt_nbv.nearest_node.locationX)
#         parent_gain = rrt_nbv.nearest_node.gain
#         parent_path_cost = rrt_nbv.distance(rrt_nbv.nearest_node, [newX, newY])
#         info_gain, path_cost = rrt_nbv.information_gain(newX, newY, theta, parent_gain, parent_path_cost)
#         print(f"Info Gain: {info_gain}")
#         rrt_nbv.add_child(rrt_nbv.nearest_node, newX, newY, info_gain)

#         # Update the best node and gain if the current node has higher gain
#         if info_gain > rrt_nbv.best_gain:
#             rrt_nbv.best_node = rrt_nbv.tree[-1]
#             rrt_nbv.best_gain = info_gain

#         plt.plot(newX, newY, 'bo', markersize=3)
#         plt.plot([rrt_nbv.nearest_node.locationX, newX], [rrt_nbv.nearest_node.locationY, newY], 'b')
#         plt.pause(0.05)

#     if len(rrt_nbv.tree) >= rrt_nbv.Nmax and rrt_nbv.best_gain > 0:
#         break

# if rrt_nbv.best_gain == 0:
#     print(f"Exploration solved with gain = 0, total nodes = {len(rrt_nbv.tree)}")
# else:
#     print("Best Gain:", rrt_nbv.best_gain)

# plt.show()

# # Plot the final explored environment
# fig_explored = plt.figure("Explored Environment")
# plt.imshow(grid, cmap='binary')
# plt.imshow(rrt_nbv.coverage, cmap='Greens', alpha=0.5)
# plt.xlabel('X-axis $(m)$')
# plt.ylabel('Y-axis $(m)$')
# plt.title("Explored Environment")
# plt.show()

import numpy as np
import matplotlib.pyplot as plt
import random

class treeNode():
    def __init__(self, locationX, locationY, gain=0.0):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None
        self.gain = gain

class RRT_NBV():
    def __init__(self, start, grid, size, theta_max, N, d_max, d_ideal, λ=0.1, Nmax=1000, NT_OL=1500, threshold=0.01):
        self.root = treeNode(start[0], start[1])
        self.grid = grid
        self.size = size
        self.theta_max = theta_max
        self.N = N
        self.d_max = d_max
        self.d_ideal = d_ideal
        self.λ = λ
        self.Nmax = Nmax
        self.NT_OL = NT_OL
        self.threshold = threshold
        self.coverage = np.zeros_like(grid)
        self.tree = [self.root]
        self.best_node = self.root
        self.best_gain = 0
        self.nearest_node = None
        self.nearest_dist = float('inf')

    def sample(self):
        x = random.randint(0, self.grid.shape[1] - 1)
        y = random.randint(0, self.grid.shape[0] - 1)
        return np.array([x, y])

    def unitVector(self, locStart, locEnd):
        v = np.array([locEnd[0] - locStart.locationX, locEnd[1] - locStart.locationY])
        return v / np.linalg.norm(v)

    def add_child(self, parent, locationX, locationY, gain):
        child_node = treeNode(locationX, locationY, gain)
        parent.children.append(child_node)
        child_node.parent = parent
        self.tree.append(child_node)
        self.updateCoverage(child_node)

    def goto(self, locStart, locEnd):
        move = self.size * self.unitVector(locStart, locEnd)
        point = np.array([locStart.locationX + move[0], locStart.locationY + move[1]])
        point[0] = min(max(0, point[0]), self.grid.shape[1] - 1)
        point[1] = min(max(0, point[1]), self.grid.shape[0] - 1)
        return point

    def Nearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist < self.nearest_dist:
            self.nearest_node = root
            self.nearest_dist = dist
        for child in root.children:
            self.Nearest(child, point)

    def inObst(self, locStart, locEnd):
        unit = self.unitVector(locStart, locEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(int(np.linalg.norm([locEnd[0] - locStart.locationX, locEnd[1] - locStart.locationY]))):
            testPoint[0] = min(self.grid.shape[1] - 1, locStart.locationX + i * unit[0])
            testPoint[1] = min(self.grid.shape[0] - 1, locStart.locationY + i * unit[1])
            if self.grid[round(testPoint[1].astype(int)), round(testPoint[0].astype(int))] == 1:
                return True
        return False

    def distance(self, node1, point):
        return np.sqrt((node1.locationX - point[0]) ** 2 + (node1.locationY - point[1]) ** 2)

    def resetNearestValues(self):
        self.nearest_node = None
        self.nearest_dist = float('inf')

    def information_gain(self, x, y, theta, parent_gain=0, parent_path_cost=0):
        total_gain = 0
        rays = self.generate_rays(x, y, theta)
        for angle, start_x, start_y in rays:
            hit = self.trace_ray(start_x, start_y, angle)
            if hit is not None:
                px, py = hit
                distance = np.sqrt((px - start_x) ** 2 + (py - start_y) ** 2)
                weight = max(0, (self.d_ideal - distance) ** 2)
                cell_value = self.grid[int(py), int(px)]
                uncertainty = 1 - abs(cell_value - 0.5) * 2
                total_gain += weight * uncertainty

        path_cost = self.distance(self.nearest_node, [x, y])
        penalized_gain = parent_gain + total_gain * np.exp(-self.λ * (parent_path_cost + path_cost))
        return penalized_gain + 1e-5, path_cost

    def generate_rays(self, x, y, theta):
        rays = []
        for i in range(self.N):
            angle = theta + (-self.theta_max + (2 * i / (self.N - 1)) * self.theta_max)
            rays.append((angle, x, y))
        return rays

    def trace_ray(self, x, y, angle):
        dx = np.cos(angle)
        dy = np.sin(angle)
        for distance in np.arange(0, self.d_max, 1):
            px = x + distance * dx
            py = y + distance * dy
            if 0 <= px < self.grid.shape[1] and 0 <= py < self.grid.shape[0]:
                cell_value = self.grid[int(py), int(px)]
                if cell_value == 1:
                    return None
                elif 0 < cell_value < 1:
                    return (px, py)
            else:
                break
        return None

    def updateCoverage(self, node):
        x, y = int(node.locationX), int(node.locationY)
        for i in range(-self.size, self.size + 1):
            for j in range(-self.size, self.size + 1):
                if 0 <= x + i < self.grid.shape[1] and 0 <= y + j < self.grid.shape[0]:
                    self.coverage[y + j, x + i] = 1

    def goalReached(self):
        return np.sum(self.coverage >= self.threshold) / np.prod(self.grid.shape) > 0.9

    def extract_best_path(self):
        path = []
        current_node = self.best_node
        while current_node is not None:
            path.append((current_node.locationX, current_node.locationY))
            current_node = current_node.parent
        path.reverse()
        return path

    def reset_tree(self):
        self.tree = [self.root]
        self.best_node = self.root
        self.best_gain = 0

    def continue_tree_construction(self):
        return len(self.tree) < self.NT_OL and self.best_gain == 0

# Set up the grid and the start position
grid = np.load('obstacle_grid8.npy')  # Load your occupancy grid here
start = np.array([100.0, 100.0])  # Starting position of the robot
size = 50
theta_max = np.pi / 4
N = 10
d_max = 50
d_ideal = 30
λ = 0.1
Nmax = 1000
NT_OL = 1500

# Set up the plot
fig = plt.figure("RRT-NBV Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

rrt_nbv = RRT_NBV(start, grid, size, theta_max, N, d_max, d_ideal, λ, Nmax, NT_OL)
plt.pause(2)

for i in range(rrt_nbv.NT_OL):
    rrt_nbv.resetNearestValues()
    print("Iteration:", i)

    point = rrt_nbv.sample()
    rrt_nbv.Nearest(rrt_nbv.root, point)

    newX, newY = rrt_nbv.goto(rrt_nbv.nearest_node, point)

    if not rrt_nbv.inObst(rrt_nbv.nearest_node, [newX, newY]):
        theta = np.arctan2(newY - rrt_nbv.nearest_node.locationY, newX - rrt_nbv.nearest_node.locationX)
        parent_gain = rrt_nbv.nearest_node.gain
        parent_path_cost = rrt_nbv.distance(rrt_nbv.nearest_node, [newX, newY])
        info_gain, path_cost = rrt_nbv.information_gain(newX, newY, theta, parent_gain, parent_path_cost)
        print(f"Info Gain: {info_gain}")
        rrt_nbv.add_child(rrt_nbv.nearest_node, newX, newY, info_gain)

        # Update the best node and gain if the current node has higher gain
        if info_gain > rrt_nbv.best_gain:
            rrt_nbv.best_node = rrt_nbv.tree[-1]
            rrt_nbv.best_gain = info_gain

        plt.plot(newX, newY, 'bo', markersize=3)
        plt.plot([rrt_nbv.nearest_node.locationX, newX], [rrt_nbv.nearest_node.locationY, newY], 'b')
        plt.pause(0.05)

    # Break condition if maximum nodes are reached or a positive gain is found
    if len(rrt_nbv.tree) >= rrt_nbv.Nmax and rrt_nbv.best_gain > 0:
        break

if rrt_nbv.best_gain == 0:
    print(f"Exploration solved with gain = 0, total nodes = {len(rrt_nbv.tree)}")
else:
    print("Best Gain:", rrt_nbv.best_gain)

plt.show()

# Plot the final explored environment
fig_explored = plt.figure("Explored Environment")
plt.imshow(grid, cmap='binary')
plt.imshow(rrt_nbv.coverage, cmap='Greens', alpha=0.5)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')
plt.title("Explored Environment")
plt.show()
