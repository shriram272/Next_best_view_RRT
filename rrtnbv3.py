# import numpy as np
# import matplotlib.pyplot as plt
# import random

# class treeNode():
#     def __init__(self, locationX, locationY):
#         self.locationX = locationX
#         self.locationY = locationY
#         self.children = []
#         self.parent = None

# class RRT_NBV():
#     def __init__(self, start, grid, size, theta_max, N, d_max, d_ideal, threshold=0.01):
#         self.randomTree = treeNode(start[0], start[1])
#         self.grid = grid
#         self.size = size
#         self.theta_max = theta_max
#         self.N = N
#         self.d_max = d_max
#         self.d_ideal = d_ideal
#         self.coverage = np.zeros_like(grid)  # To keep track of covered area
#         self.threshold = threshold  # Threshold for considering an area as "seen"
#         self.nearestNode = None
#         self.nearestDist = float('inf')

#     def sample(self):
#         x = random.randint(0, self.grid.shape[1] - 1)
#         y = random.randint(0, self.grid.shape[0] - 1)
#         return np.array([x, y])

#     def unitVector(self, locStart, locEnd):
#         v = np.array([locEnd[0] - locStart.locationX, locEnd[1] - locStart.locationY])
#         return v / np.linalg.norm(v)

#     def child(self, locationX, locationY):
#         tempNode = treeNode(locationX, locationY)
#         self.nearestNode.children.append(tempNode)
#         tempNode.parent = self.nearestNode
#         self.updateCoverage(tempNode)

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
#         if dist < self.nearestDist:
#             self.nearestNode = root
#             self.nearestDist = dist
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
#         self.nearestNode = None
#         self.nearestDist = float('inf')

#     def information_gain(self, x, y, theta):
#         total_gain = 0
#         rays = self.generate_rays(x, y, theta)
#         for angle, start_x, start_y in rays:
#             hit = self.trace_ray(start_x, start_y, angle)
#             if hit is not None:
#                 px, py = hit
#                 distance = np.sqrt((px - start_x) ** 2 + (py - start_y) ** 2)
#                 weight = (self.d_ideal - self.d_max) ** 2 - (self.d_ideal - distance) ** 2
#                 total_gain += weight
#         return total_gain

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
#                 if cell_value == 1:  # Occupied
#                     return None
#                 elif cell_value == 0.5:  # Unknown
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

# # Set up the grid and the start position
# grid = np.load('obstacle_grid6.npy')
# start = np.array([100.0, 100.0])
# size = 30  # Adjusted size for more reasonable step size
# theta_max = np.pi / 4  # ±45 degrees
# N = 10  # Number of laser readings
# d_max = 50  # Maximum distance of the sensor
# d_ideal = 30  # Ideal distance for optimal information gain

# fig = plt.figure("RRT-NBV Algorithm")
# plt.imshow(grid, cmap='binary')
# plt.plot(start[0], start[1], 'ro')
# plt.xlabel('X-axis $(m)$')
# plt.ylabel('Y-axis $(m)$')

# rrt_nbv = RRT_NBV(start, grid, size, theta_max, N, d_max, d_ideal)
# plt.pause(2)

# for i in range(1000):
#     rrt_nbv.resetNearestValues()
#     print("Iteration:", i)

#     point = rrt_nbv.sample()
#     rrt_nbv.Nearest(rrt_nbv.randomTree, point)

#     newX, newY = rrt_nbv.goto(rrt_nbv.nearestNode, point)

#     if not rrt_nbv.inObst(rrt_nbv.nearestNode, [newX, newY]):
#         rrt_nbv.child(newX, newY)
#         info_gain = rrt_nbv.information_gain(newX, newY, rrt_nbv.nearestNode.locationX)
#         print(f"Info Gain: {info_gain}")
#         plt.plot([rrt_nbv.nearestNode.locationX, newX], [rrt_nbv.nearestNode.locationY, newY], 'go', linestyle="--")
#         plt.pause(0.1)

#         if rrt_nbv.goalReached():
#             print("Sufficient area covered")
#             plt.pause(2.0)
#             break

# plt.show()



import numpy as np
import matplotlib.pyplot as plt
import random

class TreeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None

class RRT_NBV():
    def __init__(self, start, grid, size, theta_max, N, d_max, d_ideal, threshold=0.01):
        self.randomTree = TreeNode(start[0], start[1])
        self.grid = grid
        self.size = size
        self.theta_max = theta_max
        self.N = N
        self.d_max = d_max
        self.d_ideal = d_ideal
        self.coverage = np.zeros_like(grid)
        self.threshold = threshold
        self.nearestNode = None
        self.nearestDist = float('inf')

    def sample(self):
        x = random.randint(0, self.grid.shape[1] - 1)
        y = random.randint(0, self.grid.shape[0] - 1)
        return np.array([x, y])

    def unitVector(self, locStart, locEnd):
        v = np.array([locEnd[0] - locStart.locationX, locEnd[1] - locStart.locationY])
        return v / np.linalg.norm(v)

    def child(self, locationX, locationY):
        tempNode = TreeNode(locationX, locationY)
        self.nearestNode.children.append(tempNode)
        tempNode.parent = self.nearestNode
        self.updateCoverage(tempNode)

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
        if dist < self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
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
        self.nearestNode = None
        self.nearestDist = float('inf')

    def information_gain(self, x, y, theta):
        total_gain = 0
        rays = self.generate_rays(x, y, theta)
        for angle, start_x, start_y in rays:
            hit = self.trace_ray(start_x, start_y, angle)
            if hit is not None:
                px, py = hit
                distance = np.sqrt((px - start_x) ** 2 + (py - start_y) ** 2)
                weight = (self.d_ideal - self.d_max) ** 2 - (self.d_ideal - distance) ** 2
                total_gain += weight
        return total_gain

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
                elif cell_value == 0.5:
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

    def visualize_tree(self):
        def plot_branch(node):
            for child in node.children:
                plt.plot([node.locationX, child.locationX], [node.locationY, child.locationY], 'go-', linestyle="--")
                plot_branch(child)

        plt.imshow(self.grid, cmap='binary')
        plot_branch(self.randomTree)
        plt.plot(self.randomTree.locationX, self.randomTree.locationY, 'ro')
        plt.xlabel('X-axis $(m)$')
        plt.ylabel('Y-axis $(m)$')
        plt.title("Tree Expansion in RRT-NBV")
        plt.pause(0.1)

# Set up the grid and the start position
grid = np.load('obstacle_grid7.npy')
start = np.array([100.0, 100.0])
size = 30
theta_max = np.pi / 4
N = 10
d_max = 50
d_ideal = 30

fig = plt.figure("RRT-NBV Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')

rrt_nbv = RRT_NBV(start, grid, size, theta_max, N, d_max, d_ideal)
plt.pause(2)

for i in range(1000):
    rrt_nbv.resetNearestValues()
    print("Iteration:", i)

    point = rrt_nbv.sample()
    rrt_nbv.Nearest(rrt_nbv.randomTree, point)

    newX, newY = rrt_nbv.goto(rrt_nbv.nearestNode, point)

    if not rrt_nbv.inObst(rrt_nbv.nearestNode, [newX, newY]):
        rrt_nbv.child(newX, newY)
        info_gain = rrt_nbv.information_gain(newX, newY, np.arctan2(newY - rrt_nbv.nearestNode.locationY, newX - rrt_nbv.nearestNode.locationX))
        print(f"Info Gain: {info_gain}")
        plt.plot([rrt_nbv.nearestNode.locationX, newX], [rrt_nbv.nearestNode.locationY, newY], 'go-', linestyle="--")
        plt.pause(0.1)

        if rrt_nbv.goalReached():
            print("Sufficient area covered")
            plt.pause(2.0)
            break

    rrt_nbv.visualize_tree()

plt.show()
