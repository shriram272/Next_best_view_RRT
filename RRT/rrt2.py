
import numpy as np
import matplotlib.pyplot as plt
import random




class treeNode():
    def __init__(self, locationX, locationY):
        self.locationX = locationX
        self.locationY = locationY
        self.children = []
        self.parent = None


class RRT():
    def __init__(self, start, goal, numrec, grid, size):
        self.randomTree = treeNode(start[0], start[1])
        self.goal = treeNode(goal[0], goal[1])
        self.nearestNode = None
        self.iterations = min(numrec, 250)
        self.grid = grid
        self.length = size




    def sample(self):
        x = random.randint(1, grid.shape[1])
        y = random.randint(1, grid.shape[0])
        point = np.array([x, y])
        return point
    def unitVector(self, locStart, locEnd):
        v = np.array([locEnd[0] - locStart.locationX, locEnd[1] - locStart.locationY])
        unit = v / np.linalg.norm(v)
        return unit

    def child(self, locationX, locationY):
        if (locationX == self.goal.locationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent = self.nearestNode


        else:
            tempNode= treeNode(locationX,locationY)
            self.nearestNode.children.append(tempNode)
            tempNode.parent= self.nearestNode





    def goto(self, locStart, locEnd):
        move = self.length * self.unitVector(locStart, locEnd)
        point = np.array([locStart.locationX + move[0], locStart.locationY + move[1]])
        if point[0] >= grid.shape[1]:
            point[0] = grid.shape[1] - 1
        if point[1] >= grid.shape[0]:
            point[1] = grid.shape[0] - 1
        return point

    def Nearest(self, root, point):
        if not root:
            return
        dist = self.distance(root, point)
        if dist <= self.nearestDist:
            self.nearestNode = root
            self.nearestDist = dist
        for child in root.children:
            self.Nearest(child, point)

    def inObst(self, locStart, locEnd):
        unit = self.unitVector(locStart, locEnd)
        testPoint = np.array([0.0, 0.0])
        for i in range(self.length):
            testPoint[0] = min(grid.shape[1] - 1, locStart.locationX + i * unit[0])
            testPoint[1] = min(grid.shape[0] - 1, locStart.locationY + i * unit[1])
            if self.grid[round((testPoint[1]).astype(int)), round((testPoint[0]).astype(int))] == 1:
                return True
        return False

    def distance(self, node1, point):
        dist = np.sqrt((node1.locationX - point[0]) ** 2 + (node1.locationY - point[1]) ** 2)
        return dist

    def goalFound(self, point):
       if self.distance(self.goal,point) <=self.length:
         return True
       return False
    def resetNearestValues(self):
        self.nearestNode=None
        self.nearestDist= 100000






grid = np.load('obstacle_grid.npy')
start = np.array([100.0, 100.0])
goal = np.array([1600.0, 800.0])
numrec = 200
size = 200
goalRegion = plt.Circle((goal[0], goal[1]), size, color='b', fill=False)

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
ax = fig.gca()
ax.add_patch(goalRegion)
plt.xlabel('X-axis $(m)$')
plt.ylabel('Y-axis $(m)$')


rrt = RRT(start, goal, numrec, grid, size)
plt.pause(2)


for i in range(rrt.iterations):
     rrt.resetNearestValues()

     print("no of iter ", i)

     point = rrt.sample()
     rrt.Nearest(rrt.randomTree,point)

     new = rrt.goto(rrt.nearestNode, point)

     bool = rrt.inObst(rrt.nearestNode, new)
     if (bool == False):
        rrt.child(new[0],new[1])
        plt.pause(1)
        plt.plot([rrt.nearestNode.locationX,new[0]],[rrt.nearestNode.locationY,new[1]],'go',linestyle="--")

        if (rrt.goalFound(new)):
            rrt.child(goal[0],goal[1])

            print("endpoint reached")
            plt.pause(2.0)
            break


