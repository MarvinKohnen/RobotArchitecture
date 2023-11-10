import matplotlib.pyplot as plt
import numpy as np

GOAL = np.array((10, 0.))
GOAL_RADIUS = 1
CENTRAL_POSITION_OBSTACLES = np.array((0,0))
BOX_WIDTH = 6   # its the radius, not the actual width or height
BOX_HEIGHT = 6


fig, ax = plt.subplots(figsize=(10, 10))

ax.set_xlim(-10 ,10)
ax.set_ylim(-10, 10)

# Obstacles
ax.add_patch(plt.Circle((0,1), 0.2, color="r"))
ax.add_patch(plt.Circle((3,4), 0.2, color="r"))
ax.add_patch(plt.Circle((-2,4), 0.2, color="r"))
ax.add_patch(plt.Circle((2,-1), 0.2, color="r"))
ax.add_patch(plt.Circle((-2,-4), 0.2, color="r"))

# # Obstacle Border
ax.add_patch(plt.Rectangle(CENTRAL_POSITION_OBSTACLES-BOX_WIDTH, BOX_WIDTH*2, BOX_HEIGHT*2, fill = False, color="blue"))

# # Robot
ax.add_patch(plt.Circle((-9.5,0), 0.1, color="blue"))

# # Goal
ax.add_patch(plt.Circle(GOAL, GOAL_RADIUS, color="yellow"))


plt.show()