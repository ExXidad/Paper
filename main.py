import math
import sys

import Timer

import pygame
import pygame.color
from pygame.locals import *

import pymunk
from pymunk.vec2d import Vec2d
import pymunk.pygame_util

import numpy as np

from my_tuple_operations import *
from my_debug_options import MyOptions

window_size = (1500, 800)

pymunk.pygame_util.positive_y_is_up = True
pygame.init()
screen = pygame.display.set_mode(window_size)
clock = pygame.time.Clock()
font = pygame.font.Font(None, 24)
draw_options = pymunk.pygame_util.DrawOptions(screen)

my_options = MyOptions()
my_options.flags = 1

space = pymunk.Space()
space.gravity = (0.0, 0.0)


def add_ball(space, pos, velocity=(0, 0), friction=1):
    body = pymunk.Body()
    body.position = to_new_origin(pos)
    body.velocity = velocity
    shape = pymunk.Circle(body, 20)
    shape.mass = shape.area
    shape.friction = friction
    space.add(body, shape)
    return body


def add_rod(space, p1, p2, thickness=1):
    body = pymunk.Body()
    shape = pymunk.Segment(body, to_new_origin(p1), to_new_origin(p2), thickness)
    shape.mass = shape.area
    shape.friction = 5
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


def sgn(a):
    if a > 0:
        return 1
    elif a < 0:
        return -1
    elif a == 0:
        return 0


def apply_rod_scene_friction(rod, scene_friction=0.1, g_const=9.81):
    l = tuple_norm(tuple_sub(p2, p1))

    # Force acting on a mass center

    rod.force = tuple_mul(tuple_normalize(rod.velocity), -scene_friction * g_const * rod.mass)

    # Compensation torque is used to implement friction
    compensation_torque = scene_friction * rod.mass * g_const * l / 4

    rod.torque = - sgn(rod.angular_velocity) * compensation_torque


# Generating initial spiral

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
                                      0.01 * 1000000., 100000.)

# Holding the outer layer

amount_of_holding_joints = int(2 * np.pi / d_phi)
holding_joints = []
currently_holding = True

if amount_of_nodes - amount_of_holding_joints < 0:
    amount_of_holding_joints = amount_of_nodes

for i in range(amount_of_nodes - amount_of_holding_joints, amount_of_nodes):
    holding_joints.append(add_joint(space, space.static_body, rods_p1_p2[0][i],
                                    rods_p1_p2[2][i], rods_p1_p2[2][i]))

# Main loop
time_step = 1 / 10
steps_per_iteration = 60

iteration_counter = 0
iterations_to_compute = 100
iterations_to_cutoff = 0

print("Computation has started!")

exec_timer = Timer.Timer()
exec_timer.start()

expectation_timer = Timer.Timer()
expectation_timer.start()

while True:
    for event in pygame.event.get():
        if event.type == QUIT:
            exit()
        elif event.type == KEYDOWN and event.key == K_ESCAPE:
            exit()

    # Draw to the screen
    screen.fill(pygame.color.THECOLORS["white"])
    space.debug_draw(draw_options)
    pygame.display.flip()

    clock.tick(60)
    pygame.display.set_caption("fps: " + str(clock.get_fps()))

    # File output
    my_options.new_frame()
    space.debug_draw(my_options)

    # Scene iteration loop
    for i in range(steps_per_iteration):
        for rod in rods_p1_p2[0]:
            apply_rod_scene_friction(rod, scene_friction=0.)
        space.step(time_step)

    # Progress counter
    if int(iterations_to_compute / 100) == 0:
        progress_step = 1
    else:
        progress_step = int(iterations_to_compute / 100)

    if iteration_counter % progress_step == 0:
        print("Progress: " + str(iteration_counter / iterations_to_compute * 100) + "%")

        expectation_timer.stop()

        iteration_time = expectation_timer.get_elapsed_time()
        expected_time = iteration_time * (iterations_to_compute - iteration_counter) / int(iterations_to_compute / 100)

        print("Expected time left: ")
        expectation_timer.print_secs(expected_time)
        print("\n")

        expectation_timer.start()

    # Release the outer layer
    if iteration_counter == iterations_to_cutoff:
        print("Cutoff!")
        for holding_joint in holding_joints:
            space.remove(holding_joint)

    # Finish the simulation
    if iteration_counter == iterations_to_compute:
        exec_timer.stop()
        exec_timer.show_elapsed_time()
        my_options.save()
        exit()

    iteration_counter += 1
