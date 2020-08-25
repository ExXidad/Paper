import inspect
import math

import pygame
import pygame.color
from pygame.locals import *

import pymunk
from pymunk.vec2d import Vec2d
import pymunk.pygame_util

import numpy as np

from my_tuple_operations import *

pymunk.pygame_util.positive_y_is_up = True

window_size = (800, 800)

pygame.init()
screen = pygame.display.set_mode(window_size)
clock = pygame.time.Clock()
font = pygame.font.Font(None, 24)

space = pymunk.Space()
space.gravity = (0.0, 0.0)
draw_options = pymunk.pygame_util.DrawOptions(screen)


def add_ball(space, pos, velocity=(0, 0), friction=1):
    body = pymunk.Body()
    body.position = to_new_origin(pos)
    body.velocity = velocity
    shape = pymunk.Circle(body, 20)
    shape.mass = shape.area
    shape.friction = friction
    space.add(body, shape)
    return body


def add_bar(space, pos):
    body = pymunk.Body()
    body.position = Vec2d(pos)
    shape = pymunk.Segment(body, (20, 40), (-30, -40), 2)
    shape.mass = 2
    shape.friction = 0.7
    space.add(body, shape)
    return body


def add_lever(space, pos, box_offset):
    body = pymunk.Body()
    body.position = pos + Vec2d(box_offset) + (0, -20)
    shape = pymunk.Segment(body, (0, 20), (0, -20), 5)
    shape.mass = 1
    shape.friction = 0.7
    space.add(body, shape)
    return body


def add_rod(space, p1, p2, thickness=1):
    body = pymunk.Body()
    # body.position = tuple_sum(p1, tuple_mul(tuple_sub(p2, p1), 0.5))
    shape = pymunk.Segment(body, to_new_origin(p1), to_new_origin(p2), thickness)
    shape.mass = shape.area
    shape.friction = 0.7
    space.add(body, shape)
    return body


newOrigin = tuple_mul(window_size, 0.5)


def to_new_origin(pt):
    return tuple_sum(pt, newOrigin)


def gen_i_node_of_spiral_coord(i, r_init, d_r, phi_init, d_phi):
    return polar_to_cartesian(r_init + i * d_r, phi_init + i * d_phi)


def rest_angle(p_i_minus_1, p_i, p_i_plus_1):
    return np.pi / 2 - np.arctan2(p_i_plus_1[1] - p_i[1], p_i_plus_1[0] - p_i[0]) - \
           np.arctan2(p_i[0] - p_i_minus_1[1], p_i[0] - p_i_minus_1[1])


def add_joint(space, body1, body2, p1, p2, colliding=True):
    joint = pymunk.PinJoint(body1, body2, to_new_origin(p1), to_new_origin(p2))
    joint.collide_bodies = colliding
    space.add(joint)
    return joint


def add_damped_rotary_spring(space, body1, body2, rest_angle, stiffness, damping):
    damped_rotatory_spring = pymunk.DampedRotarySpring(body1, body2, rest_angle, stiffness, damping)
    space.add(damped_rotatory_spring)
    return damped_rotatory_spring


# rod = add_rod(space, to_new_origin((0, 0)), to_new_origin((100, 200)))
# rod.velocity = (40, 20)
#
# add_ball(space, to_new_origin((0, 0)))

# x_axis = pymunk.Segment(space.static_body, to_new_origin((-100, 0)), to_new_origin((100, 0)), 1)
# y_axis = pymunk.Segment(space.static_body, to_new_origin((0, -100)), to_new_origin((0, 100)), 1)
# x_axis.friction = 1
# y_axis.friction = 1
# x_axis.elasticity = 1
# y_axis.elasticity = 1
# space.add(x_axis, y_axis)

# add_ball(space, (0, 0), box_offset)

amount_of_nodes = 300
phi_init = 0
d_phi = np.deg2rad(10)

r_init = 160
d_r = d_phi

rods_p1_p2 = [[], [], []]

for i in range(amount_of_nodes):
    p1 = gen_i_node_of_spiral_coord(i, r_init, d_r, phi_init, d_phi)
    p2 = gen_i_node_of_spiral_coord(i + 1, r_init, d_r, phi_init, d_phi)

    rods_p1_p2[0].append(add_rod(space, p1, p2))
    rods_p1_p2[1].append(p1)
    rods_p1_p2[2].append(p2)

for i in range(amount_of_nodes - 1):
    joint = add_joint(space, rods_p1_p2[0][i], rods_p1_p2[0][i + 1],
                      rods_p1_p2[2][i], rods_p1_p2[1][i + 1], colliding=False)
    spring = add_damped_rotary_spring(space, rods_p1_p2[0][i + 1], rods_p1_p2[0][i],
                                      -d_phi,
                                      1000000., 100000.)

# ball1 = add_ball(space, (0, 300), velocity=(0, -20 * 0))

# rod1 = add_rod(space, (0, 0), (0, 200))
# rod2 = add_rod(space, (0, 200), (200, 200))
#
# joint = add_joint(space, rod1, rod2, (0, 200), (0, 200), False)
#
# rotatory_spring = pymunk.DampedRotarySpring(rod2, rod1, np.deg2rad(90), 1000000., 1000000.)
#
# space.add(rotatory_spring)
#
# rod1.angular_velocity = 1
# rod1.velocity = (-30, 0)


time_step = 1 / 60

steps_per_iteration = 5

simulation_is_paused = False

while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            exit()
        elif event.type == KEYDOWN and event.key == K_ESCAPE:
            exit()
        elif event.type == KEYDOWN and event.key == K_SPACE:
            simulation_is_paused = not simulation_is_paused

    screen.fill(pygame.color.THECOLORS["white"])

    if not simulation_is_paused:
        for i in range(steps_per_iteration):
            space.step(time_step)

    space.debug_draw(draw_options)
    pygame.display.flip()

    clock.tick(60)
    pygame.display.set_caption("fps: " + str(clock.get_fps()))
