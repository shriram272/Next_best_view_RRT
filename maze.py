import numpy as np
import matplotlib.pyplot as plt

# Create a 2000x2000 obstacle map
obstacle_map = np.ones((2000, 2000))

# Add initial obstacles
obstacle_map[12:600, 1300:2000] = 0.5
obstacle_map[1000:1599, 1177:1699] = 0.6
obstacle_map[1000:800, 100:900] = 0.8

# Add maze-like obstacles
def add_obstacle(x_start, y_start, x_end, y_end):
    obstacle_map[x_start:x_end, y_start:y_end] = 0.7

# Vertical walls
for x in range(100, 2000, 200):
    add_obstacle(0, x, 2000, x + 120)

# Horizontal walls
for y in range(100, 2000, 200):
    add_obstacle(y, 0, y + 120, 2000)

# Add some random obstacles
for _ in range(50):
    x_start = np.random.randint(0, 1900)
    y_start = np.random.randint(0, 1900)
    width = np.random.randint(10, 100)
    height = np.random.randint(10, 100)
    add_obstacle(x_start, y_start, min(x_start + width, 2000), min(y_start + height, 2000))

# Save the obstacle map
np.save('obstacle_grid8.npy', obstacle_map)

# Display the obstacle map
plt.imshow(obstacle_map, cmap='gray')
plt.show()
