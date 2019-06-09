import sys
from robot import Robot
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
import time
import helper
import math

# Antecedents objects
distancesAntecedents = []
for i in range(8):
    distancesAntecedents.append(ctrl.Antecedent(np.arange(0, 5, 0.01), 'distance_' + str(i)))

direction = ctrl.Antecedent(np.arange(-math.pi, math.pi, 0.01), 'direction')
direction['error_left'] = fuzz.trimf(direction.universe, [-math.pi, -math.pi, 0])
direction['error_right'] = fuzz.trimf(direction.universe, [0, math.pi, math.pi])
direction['no_error'] = fuzz.trimf(direction.universe, [-0.2, 0, 0.2])
goal_distance = ctrl.Antecedent(np.arange(0, 500, 0.01), 'goal_distance')
goal_distance['perto'] = fuzz.trimf(goal_distance.universe, [0, 0, 0.2])
goal_distance['longe'] = fuzz.trapmf(goal_distance.universe, [0.2, 1, 500, 500])

for i in range(8):
    distancesAntecedents[i]["perto"] = fuzz.trapmf(distancesAntecedents[i].universe, [0, 0, 0.8, 2])
    distancesAntecedents[i]["longe"] = fuzz.trapmf(distancesAntecedents[i].universe, [0.8, 2, 6, 6])

vel_left = ctrl.Consequent(np.arange(-4, 4, 0.1), 'vel_left')
vel_right = ctrl.Consequent(np.arange(-4, 4, 0.1), 'vel_right')

vel_left['pos_fast'] = fuzz.trimf(vel_left.universe, [1, 5, 5])
vel_left['pos_slow'] = fuzz.trimf(vel_left.universe, [0, 1.5, 1.5])
vel_left['neg_fast'] = fuzz.trimf(vel_left.universe, [-5, -5, -1])
vel_left['neg_slow'] = fuzz.trimf(vel_left.universe, [-1.5, -1.5, 0])
vel_right['pos_fast'] = fuzz.trimf(vel_left.universe, [1, 4.8, 4.8])
vel_right['pos_slow'] = fuzz.trimf(vel_left.universe, [0, 1.3, 1.3])
vel_right['neg_fast'] = fuzz.trimf(vel_left.universe, [-5, -5, -1])
vel_right['neg_slow'] = fuzz.trimf(vel_left.universe, [-1.5, -1.5, 0])

rule1 = ctrl.Rule(direction['error_left'] & goal_distance['longe'], vel_left['neg_fast'])
rule2 = ctrl.Rule(direction['error_left'] & goal_distance['longe'], vel_right['pos_fast'])
rule3 = ctrl.Rule(direction['error_right'] & goal_distance['longe'], vel_right['neg_fast'])
rule4 = ctrl.Rule(direction['error_right'] & goal_distance['longe'], vel_left['pos_fast'])
rule5 = ctrl.Rule(direction['no_error'] & goal_distance['longe'], vel_left['pos_fast'])
rule6 = ctrl.Rule(direction['no_error'] & goal_distance['longe'], vel_right['pos_fast'])
rule7 = ctrl.Rule(goal_distance['perto'], vel_right['pos_slow'])
rule8 = ctrl.Rule(goal_distance['perto'], vel_right['neg_slow'])
rule9 = ctrl.Rule(goal_distance['perto'], vel_left['neg_slow'])
rule10 = ctrl.Rule(goal_distance['perto'], vel_left['pos_slow'])

rule11 = ctrl.Rule(distancesAntecedents[3]['perto'] | distancesAntecedents[4]['perto'], vel_left['pos_fast'])
rule12 = ctrl.Rule(distancesAntecedents[3]['perto'] | distancesAntecedents[4]['perto'], vel_right['neg_fast'])
rule13 = ctrl.Rule(distancesAntecedents[2]['perto'], vel_right['neg_slow'])
rule14 = ctrl.Rule(distancesAntecedents[2]['perto'], vel_left['pos_slow'])
rule15 = ctrl.Rule(distancesAntecedents[5]['perto'], vel_right['pos_slow'])
rule16 = ctrl.Rule(distancesAntecedents[5]['perto'], vel_left['neg_slow'])
rule17 = ctrl.Rule(distancesAntecedents[2]['longe'] & distancesAntecedents[3]['longe'] & distancesAntecedents[4]['longe'] & distancesAntecedents[5]['longe'], vel_right['pos_fast'])
rule18 = ctrl.Rule(distancesAntecedents[2]['longe'] & distancesAntecedents[3]['longe'] & distancesAntecedents[4]['longe'] & distancesAntecedents[5]['longe'], vel_left['pos_fast'])

go_to_goal_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10])
go_to_goal = ctrl.ControlSystemSimulation(go_to_goal_ctrl)

avoid_obstacle_ctrl = ctrl.ControlSystem([rule11, rule12, rule13, rule14, rule15, rule16, rule17, rule18])
avoid_obstacle = ctrl.ControlSystemSimulation(avoid_obstacle_ctrl)

def fuzzyGoToGoal(error, distance):
    go_to_goal.input['direction'] = error
    go_to_goal.input['goal_distance'] = distance
    go_to_goal.compute()
    return [go_to_goal.output['vel_left'], go_to_goal.output['vel_right']]

def fuzzyAvoidObstacle(dist):
    # print('dist:', dist)
    for i in range(2, 6):
        avoid_obstacle.input['distance_' + str(i)] = dist[i]
    avoid_obstacle.compute()
    return [avoid_obstacle.output['vel_left'], avoid_obstacle.output['vel_right']]

goal = [-4.7, 2.3]

def isClose(distances):
    for i, d in enumerate(distances):
        if i == 0 or i == 7:
            continue
        if (i == 3 or i == 4) and d < 1.5:
            return True
        if (i == 1 or i == 6) and d < 0.5:
            return True
        if d < 1:
            return True
    return False

robot = Robot()
while(robot.get_connection_status() != -1):
    us_distances = robot.read_ultrassonic_sensors()
    if isClose(us_distances[:8]):
        print("Close")
        vel = fuzzyAvoidObstacle(us_distances[:8])    
    else:
        print("Go to goal")
        orientation = robot.get_current_orientation()
        position = robot.get_current_position()
        angle = helper.diff_angle(goal, position)

        diff = orientation[2] - angle + math.pi
        if diff > math.pi:
            diff = 2*math.pi - diff
            diff *= -1
        if diff < -math.pi:
            diff += 2*math.pi

        distance = helper.euclidian_distance(position[:2], goal)

        vel = fuzzyGoToGoal(diff, distance) #Using only the 8 frontal sensors

    # print(vel)
    robot.set_left_velocity(vel[0])
    robot.set_right_velocity(vel[1])
