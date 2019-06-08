import math
import numpy as np

def diff_angle(v1, v2):
    dx = v2[0] - v1[0]
    dy = v2[1] - v1[1]
    return math.atan2(dy, dx)

# v1 = np.array([1,0])
# v2 = np.array([0,0])
# 
# 
# print(diff_angle(v1, v2))
