import numpy as np
import pymunk
from pymunk.vec2d import Vec2d


class MyOptions(pymunk.SpaceDebugDrawOptions):
    data = []

    last_p2 = None

    def append_last_p2(self):
        self.data[-1].append(self.last_p2)

    def new_frame(self):
        if self.last_p2 is not None:
            self.append_last_p2()
        self.data.append([])

    #
    # def draw_shape(self, shape):
    #     self.data[-1].append(shape.a)
    #     print(shape.a)

    def save(self):
        self.append_last_p2()
        np.save("data", self.data)

    def draw_fat_segment(self, *args):
        self.data[-1].append(args[0])
        self.last_p2 = args[1]
