import torch
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from lemniscate import Lemniscate
import numpy as np


# Create a figure and 3D axis
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_title('3D Lemniscate Curve Over Time')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim(-1.5, 1.5)
ax.set_ylim(-1.5, 1.5)
ax.set_zlim(-1.5, 1.5)

# Initialize the plot
# line, = ax.plot([], [], [], label='Lemniscate Curve')
# point, = ax.plot([], [], [], 'ro')  # Point representing the current time

l = Lemniscate(scale=2)
t_values = torch.linspace(0, 20*np.pi, 1000)  # Vary the time parameter
pos = l.update(t_values).numpy()


# 绘制 Lemniscate
# plt.plot(pos[:,0],pos[:,1], label=f'Lemniscate')
ax.plot(pos[:,0],pos[:,1],pos[:,2])
plt.title('Lemniscate of Bernoulli')
# plt.xlabel('x')
# plt.ylabel('y')
# plt.zlabel('z')
# plt.grid(True)
# plt.axhline(0, color='black',linewidth=0.5)
# plt.axvline(0, color='black',linewidth=0.5)
# plt.legend()
plt.show()


# def init():
#     line.set_data([], [])
#     line.set_3d_properties([])
#     point.set_data([], [])
#     point.set_3d_properties([])
#     return line, point

# def update(frame):
#     t_values = torch.linspace(0, frame / 10.0, 1000)  # Vary the time parameter
#     # c_value = 1.0
#     # x, y, z = lemniscate(t_values, c_value)

#     pos = l.update(t_values).numpy()
#     print(t_values)
#     print(pos)

#     print(pos[:,0])
#     line.set_data(pos[:,0], pos[:,1])
#     line.set_3d_properties(pos[:,2])
#     point.set_data(pos[:,-1].item(), pos[:,-1].item())
#     point.set_3d_properties(pos[:,-1].item())  # Show the current position in red
#     return line, point

# # Animate the plot
# ani = FuncAnimation(fig, update, frames=100, init_func=init, blit=True)
# plt.legend()
# plt.show()
