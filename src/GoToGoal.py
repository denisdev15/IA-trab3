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
direction = ctrl.Antecedent(np.arange(-math.pi, math.pi, 0.01), 'direction')
direction['error_left'] = fuzz.trimf(direction.universe, [-math.pi, -math.pi, 0])
direction['error_right'] = fuzz.trimf(direction.universe, [0, math.pi, math.pi])
direction['no_error'] = fuzz.trimf(direction.universe, [-0.2, 0, 0.2])

goal_distance = ctrl.Antecedent(np.arange(0, 500, 0.01), 'goal_distance')
goal_distance['perto'] = fuzz.trimf(goal_distance.universe, [0, 0, 0])
goal_distance['longe'] = fuzz.trapmf(goal_distance.universe, [0, 1, 500, 500])

vel_left = ctrl.Consequent(np.arange(-4, 4, 0.1), 'vel_left')
vel_right = ctrl.Consequent(np.arange(-4, 4, 0.1), 'vel_right')

vel_left['pos'] = fuzz.trimf(vel_left.universe, [0, math.pi, math.pi])
vel_left['neg'] = fuzz.trimf(vel_left.universe, [-math.pi, -math.pi, 0])
vel_right['pos'] = fuzz.trimf(vel_left.universe, [0, math.pi, math.pi])
vel_right['neg'] = fuzz.trimf(vel_left.universe, [-math.pi, -math.pi, 0])

rule1 = ctrl.Rule(direction['error_left'] & goal_distance['longe'], vel_left['neg'])
rule2 = ctrl.Rule(direction['error_left'] & goal_distance['longe'], vel_right['pos'])
rule3 = ctrl.Rule(direction['error_right'] & goal_distance['longe'], vel_right['neg'])
rule4 = ctrl.Rule(direction['error_right'] & goal_distance['longe'], vel_left['pos'])
rule5 = ctrl.Rule(direction['no_error'] & goal_distance['longe'], vel_left['pos'])
rule6 = ctrl.Rule(direction['no_error'] & goal_distance['longe'], vel_right['pos'])
rule7 = ctrl.Rule(goal_distance['perto'], vel_right['pos'])
rule8 = ctrl.Rule(goal_distance['perto'], vel_right['neg'])
rule9 = ctrl.Rule(goal_distance['perto'], vel_left['neg'])
rule10 = ctrl.Rule(goal_distance['perto'], vel_left['pos'])

go_to_goal_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10])
go_to_goal = ctrl.ControlSystemSimulation(go_to_goal_ctrl)

positions = [[], []]
velocities_left = []
velocities_right = []
distances = []

def get_info_to_plot(vel, distance, it):
    if it % 150 == 0:
        position = robot.get_current_position()
        positions[0].append(position[0])
        positions[1].append(position[1])
        velocities_left.append(vel[0])
        velocities_right.append(vel[1])
        distances.append(distance)

def fuzzy(error, distance):
    go_to_goal.input['direction'] = error
    go_to_goal.input['goal_distance'] = distance
    go_to_goal.compute()
    return [go_to_goal.output['vel_left'], go_to_goal.output['vel_right']]

goal = [0, 0]

i=0
robot = Robot()
while(robot.get_connection_status() != -1):
    orientation = robot.get_current_orientation()
    position = robot.get_current_position()
    angle = helper.diff_angle(goal, position)

    error = orientation[2] - angle + math.pi
    if error > math.pi:
        error = 2*math.pi - error
        error *= -1
    if error < -math.pi:
        error += 2*math.pi

    distance = helper.euclidian_distance(position[:2], goal)
    vel = fuzzy(error, distance) #Using only the 8 frontal sensors
    get_info_to_plot(vel, distance, i)
    robot.set_left_velocity(vel[0])
    robot.set_right_velocity(vel[1])
    i += 1

plt.plot(positions[0], positions[1])
plt.plot(positions[0][0], positions[1][0], 'go', label="Posição Inicial")
plt.plot(goal[0], goal[1], 'gx', label="Objetivo")
plt.plot(positions[0][-1], positions[1][-1], 'rx', label="Posição Final")
plt.title('Posição do robô')
plt.legend()
plt.show()

plt.plot(velocities_left, 'b', label='Motor Esquerdo')
plt.plot(velocities_right, 'r', label='Motor Direito')
plt.title('Velocidade dos motores')
plt.legend()
plt.show()

plt.plot(distances)
plt.title('Distância para o objetivo')
plt.show()
