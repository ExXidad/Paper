import pymunk
import matplotlib.pyplot as plt
import pymunk.matplotlib_util
from my_tuple_operations import *

space = pymunk.Space()
space.gravity = (0, -9.81)

window_size = (1500, 800)
newOrigin = tuple_mul(window_size, 0.5)

fig = plt.figure(figsize=(14, 10))
ax = plt.axes(xlim=(0, 1000), ylim=(0, 700))
ax.set_aspect("equal")

options = pymunk.matplotlib_util.DrawOptions(ax)


def add_ball(space, pos, velocity=(0, 0), friction=1):
    body = pymunk.Body()
    body.position = pos
    body.velocity = velocity
    shape = pymunk.Circle(body, 20)
    shape.mass = shape.area
    shape.friction = friction
    space.add(body, shape)
    return body


ball = add_ball(space, (0, 0))

space.debug_draw(options)

fig.show()
