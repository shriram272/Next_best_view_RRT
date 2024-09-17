# RRT
An implementation of path planning algo of Rapidly exploring random trees made using python and matplotlib.
There are two versions of code a simple RRT and RRT with next best view planner logic.

1. RRT
Working and Results-

1. Class Definitions
TreeNode Class

    Purpose: Represents a node in the RRT (Rapidly-exploring Random Tree) with a location and pointers to its children and parent.
    Attributes:
        locationX and locationY: Coordinates of the node.
        children: List of child nodes.
        parent: Parent node.

RRT Class

    Purpose: Implements the RRT algorithm to explore the space and find a path from the start to the goal.
    Attributes:
        randomTree: Root node of the RRT tree (starting point).
        goal: Goal node.
        nearestNode: Nearest node in the tree to the current random sample.
        nearestDist: Distance to the nearest node.
        iterations: Maximum number of iterations for the RRT algorithm.
        grid: 2D grid representing the environment with obstacles.
        length: Step size for expanding the tree.

2. RRT Methods
sample

    Purpose: Generates a random point within the grid bounds.
    Logic: Randomly selects x and y coordinates using random.randint.

unitVector

    Purpose: Computes the unit vector from locStart to locEnd.
    Logic: Finds the direction vector and normalizes it to have a magnitude of 1.

addChild

    Purpose: Adds a new node as a child of the nearest node.
    Logic: If the new node is the goal, it sets the goal as a child of the nearest node. Otherwise, it creates a new node and attaches it to the nearest node.

goto

    Purpose: Computes the next point by moving from locStart towards locEnd with a step size defined by length.
    Logic: Calculates the point using the unit vector and ensures it stays within grid bounds.

Nearest

    Purpose: Finds the nearest node in the tree to a given point.
    Logic: Recursively traverses the tree to find the node with the smallest distance to the point.

inObst

    Purpose: Checks if a line segment from locStart to locEnd intersects with any obstacles in the grid.
    Logic: Samples points along the segment and checks if they fall within obstacles.

distance

    Purpose: Computes the Euclidean distance between a node and a point.
    Logic: Uses the Euclidean distance formula.

goalFound

    Purpose: Checks if the goal is within the step size distance from a given point.
    Logic: Compares the distance between the point and the goal with length.

resetNearestValues

    Purpose: Resets the nearest node and its distance.
    Logic: Sets nearestNode to None and nearestDist to a very large number (float('inf')).
    
[Screencast from 19-08-24 08:14:49 PM IST.webm](https://github.com/user-attachments/assets/93735092-fad3-4caf-9dea-846ca708194f)





2. RRT with Next Best View Planner

The proposed planner employs a receding horizon “next-best-view” scheme: In an online computed random tree it finds the best branch, the quality of which is determined by the amount of unmapped space that can be explored. Only the first edge of this branch is executed at every planning step, while repetition of this procedure leads to complete exploration results. The proposed planner is capable of running online, onboard a robot with limited resources
   
Working and Results- 
New RRT-NBV Class

The RRT_NBV class introduces several enhancements over the basic RRT algorithm:

    Attributes:
        λ (Lambda): A parameter for penalizing the path cost in the information gain calculation.
        Nmax, NT_OL, threshold: Parameters controlling the maximum number of nodes, the upper limit for the number of nodes to construct the tree, and the threshold for determining coverage respectively.
        coverage: A grid representing the explored area, updated as nodes are added to the tree.

    Methods:
        information_gain: Calculates the gain of a potential new node based on its surroundings. It uses rays cast from the node to estimate how much new information would be gained by exploring that node.
        generate_rays: Creates rays from a node to estimate the information gain by evaluating different angles.
        trace_ray: Traces each ray to determine if it hits an obstacle or provides useful information.
        updateCoverage: Updates the coverage grid to reflect the area explored around a node.
        goalReached: Checks if the coverage has reached a certain threshold, indicating significant exploration.
        extract_best_path: Retrieves the path from the root to the best node found.
        reset_tree: Resets the tree to its initial state.
        continue_tree_construction: Determines if the tree construction should continue based on the number of nodes and the best gain.
[Screencast from 08-08-24 08:51:16 PM IST.webm](https://github.com/user-attachments/assets/c9e7a5de-237d-4f51-802a-63d5b9b27b21)
[Screencast from 13-08-24 12:37:38 AM IST.webm](https://github.com/user-attachments/assets/c588c471-bb3c-4b1e-83a8-4e6ac314b64f)
