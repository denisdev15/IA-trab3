import math
import numpy as np

def diff_angle(v1, v2):
    dx = v2[0] - v1[0]
    dy = v2[1] - v1[1]
    return math.atan2(dy, dx)

def euclidian_distance(x, y):
    return math.sqrt(sum([(a - b) ** 2 for a, b in zip(x, y)]))
