import numpy as np
import matplotlib.pyplot as plt

# Create a 10x10 obstacle map
obstacle_map = np.zeros((2000, 2000))

# Add obstacles to the map
obstacle_map[12:600, 1300:2000] = 1
obstacle_map[1000:1599, 1177:1699] = 1
obstacle_map[1000:800, 100:900] = 1

# Display the obstacle map
np.save('obstacle_grid.npy', obstacle_map)
#print(obstacle_map)
#plt.imshow(obstacle_map, cmap='gray')
#plt.show()
