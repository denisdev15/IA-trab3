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
goal_distance['perto'] = fuzz.trimf(goal_distance.universe, [0, 0, 0.05])
goal_distance['longe'] = fuzz.trapmf(goal_distance.universe, [0.05, 1, 500, 500])

for i in range(8):
    distancesAntecedents[i]["perto"] = fuzz.trapmf(distancesAntecedents[i].universe, [0, 0, 0.5, 1.5])
    distancesAntecedents[i]["longe"] = fuzz.trapmf(distancesAntecedents[i].universe, [0.5, 1.5, 6, 6])

vel_left = ctrl.Consequent(np.arange(-4, 4, 0.1), 'vel_left')
vel_right = ctrl.Consequent(np.arange(-4, 4, 0.1), 'vel_right')

vel_left['pos_fast'] = fuzz.trimf(vel_left.universe, [1, 4, 4])
vel_left['pos_slow'] = fuzz.trimf(vel_left.universe, [0, 1.5, 1.5])
vel_left['neg_fast'] = fuzz.trimf(vel_left.universe, [-4, -4, -1])
vel_left['neg_slow'] = fuzz.trimf(vel_left.universe, [-1.5, -1.5, 0])
vel_right['pos_fast'] = fuzz.trimf(vel_left.universe, [1, 4, 4])
vel_right['pos_slow'] = fuzz.trimf(vel_left.universe, [0, 1.5, 1.5])
vel_right['neg_fast'] = fuzz.trimf(vel_left.universe, [-4, -4, -1])
vel_right['neg_slow'] = fuzz.trimf(vel_left.universe, [-1.5, -1.5, 0])

# go to goal rules
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

# Avoid obstacle rules
rule11 = ctrl.Rule(distancesAntecedents[1]['perto'] | distancesAntecedents[2]['perto'] | distancesAntecedents[3]['perto'], vel_left['pos_fast'])
rule12 = ctrl.Rule(distancesAntecedents[1]['perto'] | distancesAntecedents[2]['perto'] | distancesAntecedents[3]['perto'], vel_right['neg_fast'])
rule13 = ctrl.Rule(distancesAntecedents[4]['perto'] | distancesAntecedents[5]['perto'] | distancesAntecedents[6]['perto'], vel_right['pos_fast'])
rule14 = ctrl.Rule(distancesAntecedents[4]['perto'] | distancesAntecedents[5]['perto'] | distancesAntecedents[6]['perto'], vel_left['neg_fast'])
rule15 = ctrl.Rule(distancesAntecedents[1]['longe'] & distancesAntecedents[2]['longe'] & distancesAntecedents[3]['longe'] & distancesAntecedents[4]['longe'] & distancesAntecedents[5]['longe'] & distancesAntecedents[6]['longe'] & goal_distance['longe'], vel_right['pos_fast'])
rule16 = ctrl.Rule(distancesAntecedents[1]['longe'] & distancesAntecedents[2]['longe'] & distancesAntecedents[3]['longe'] & distancesAntecedents[4]['longe'] & distancesAntecedents[5]['longe'] & distancesAntecedents[6]['longe'] & goal_distance['longe'], vel_left['pos_fast'])

go_to_goal_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9, rule10, rule11, rule12, rule13, rule14, rule15, rule16])
go_to_goal = ctrl.ControlSystemSimulation(go_to_goal_ctrl)

positions = [[], []]
velocities_left = []
velocities_right = []
distances = []
direction_error = []

def get_info_to_plot(vel, distance, position, error, it):
    if it % 250 == 0:
        positions[0].append(position[0])
        positions[1].append(position[1])
        velocities_left.append(vel[0])
        velocities_right.append(vel[1])
        distances.append(distance)
        direction_error.append(error)


def fuzzy(error, goal_distance, distances):
    go_to_goal.input['direction'] = error
    go_to_goal.input['goal_distance'] = goal_distance
    for i in range(1, 7):
        go_to_goal.input['distance_' + str(i)] = distances[i]
    go_to_goal.compute()
    return [go_to_goal.output['vel_left'], go_to_goal.output['vel_right']]

# goal = [0, 0]
x = float(input("Insira as coordenadas de objetivo do robô.\nX: "))
y = float(input("Y: "))
goal = (x, y)
print(goal)

robot = Robot()
i=0
while(robot.get_connection_status() != -1):
    us_distances = robot.read_ultrassonic_sensors()
    position = robot.get_current_position()
    orientation = robot.get_current_orientation()
    distance = helper.euclidian_distance(position[:2], goal)
    angle = helper.diff_angle(goal, position)
    error = orientation[2] - angle + math.pi
    if error > math.pi:
        error = 2*math.pi - error
        error *= -1
    if error < -math.pi:
        error += 2*math.pi

    vel = fuzzy(error, distance, us_distances[:8])
    get_info_to_plot(vel, distance, position, error, i)

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
plt.title('Velocidade dos motores (rad/s) por iteração')
plt.legend()
plt.show()

plt.plot(distances)
plt.title('Distância para o objetivo (m)')
plt.show()

fig, ax1 = plt.subplots()

color = 'tab:red'
ax1.set_xlabel('Iterações')
ax1.set_ylabel('Distância para o objetivo (m)', color=color)
ax1.plot(distances, color=color)
ax1.tick_params(axis='y', labelcolor=color)

ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
color = 'tab:blue'
ax2.set_ylabel('Erro na direção para o objetivo (radianos)', color=color)  # we already handled the x-label with ax1
ax2.plot(direction_error, color=color)
ax2.tick_params(axis='y', labelcolor=color)

fig.tight_layout()  # otherwise the right y-label is slightly clipped
plt.show()
