import sys
from robot import Robot

def fuzzy(dist, vel):
    # TODO fuzzy logic
    return [10, 10]

robot = Robot()
while(robot.get_connection_status() != -1):
    us_distances = robot.read_ultrassonic_sensors()
    vel = fuzzy(us_distances[:8], 3) #Using only the 8 frontal sensors
    robot.set_left_velocity(vel[0])
    robot.set_right_velocity(vel[1])
