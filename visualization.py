import matplotlib.pyplot as plt
from matplotlib import animation
from my_tuple_operations import *
import numpy as np

window_size = (1500, 800)
newOrigin = tuple_mul(window_size, 0.5)

fig = plt.figure(figsize=(14, 10))
ax = plt.axes(xlim=(0, window_size[0]), ylim=(0, window_size[1]))
ax.set_aspect("equal")
line, = ax.plot([], [])


# initialization function: plot the background of each frame
def init():
    line.set_data([], [])
    return line,


# animation function.  This is called sequentially
def animate(i):
    x = data[i, :, 0]
    y = data[i, :, 1]
    line.set_data(x, y)
    return line,


data = np.load("data.npy")

anim = animation.FuncAnimation(fig, animate, init_func=init,
                               frames=len(data), interval=20, blit=True)

plt.show()
