import numpy as np


def tuple_sum(t1, t2):
    return tuple(map(sum, zip(t1, t2)))


def tuple_sub(t1, t2):
    result = []
    for i in range(min(len(t1), len(t2))):
        result.append(t1[i] - t2[i])
    return tuple(result)


def tuple_mul(t, a):
    result = list(t)
    for i in range(len(t)):
        result[i] *= a
    return tuple(result)


def tuple_norm(t):
    sum_sq = 0
    for element in t:
        sum_sq += element ** 2
    return np.sqrt(sum_sq)


def polar_to_cartesian(r, phi):
    return (r * np.cos(phi), r * np.sin(phi))
