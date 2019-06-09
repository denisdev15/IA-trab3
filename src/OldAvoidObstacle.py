import sys
from robot import Robot
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

# Antecedents objects
distancesAntecedents = []
for i in range(8):
    distancesAntecedents.append(ctrl.Antecedent(np.arange(0, 5, 0.01), 'distance_' + str(i)))
vel_left = ctrl.Consequent(np.arange(-5, 5, 0.1), 'vel_left')
vel_right = ctrl.Consequent(np.arange(-5, 5, 0.1), 'vel_right')

for i in range(8):
    distancesAntecedents[i]["perto"] = fuzz.trapmf(distancesAntecedents[i].universe, [0, 0, 0.8, 2.5])
    distancesAntecedents[i]["longe"] = fuzz.trapmf(distancesAntecedents[i].universe, [0.8, 2.5, 6, 6])
    # distancesAntecedents[i]["perto"].view()
    # # distancesAntecedents[i]["longe"].view()
    # plt.show()

vel_left['pos'] = fuzz.trimf(vel_left.universe, [0, 3, 3])
vel_left['neg'] = fuzz.trimf(vel_left.universe, [-3, -3, 0])
vel_right['pos'] = fuzz.trimf(vel_left.universe, [0, 2.8, 2.8])
vel_right['neg'] = fuzz.trimf(vel_left.universe, [-3, -3, 0])

rule1 = ctrl.Rule(distancesAntecedents[0]['perto'] | distancesAntecedents[1]['perto'] | distancesAntecedents[2]['perto'] | distancesAntecedents[3]['perto'], vel_right['neg'])
rule2 = ctrl.Rule(distancesAntecedents[0]['perto'] | distancesAntecedents[1]['perto'] | distancesAntecedents[2]['perto'] | distancesAntecedents[3]['perto'], vel_left['pos'])
rule3 = ctrl.Rule(distancesAntecedents[4]['perto'] | distancesAntecedents[5]['perto'] | distancesAntecedents[6]['perto'] | distancesAntecedents[7]['perto'], vel_right['pos'])
rule4 = ctrl.Rule(distancesAntecedents[4]['perto'] | distancesAntecedents[5]['perto'] | distancesAntecedents[6]['perto'] | distancesAntecedents[7]['perto'], vel_left['neg'])
rule5 = ctrl.Rule(distancesAntecedents[0]['longe'] & distancesAntecedents[1]['longe'] & distancesAntecedents[2]['longe'] & distancesAntecedents[3]['longe'] & distancesAntecedents[4]['longe'] & distancesAntecedents[5]['longe'] & distancesAntecedents[6]['longe'] & distancesAntecedents[7]['longe'], vel_right['pos'])
rule6 = ctrl.Rule(distancesAntecedents[0]['longe'] & distancesAntecedents[1]['longe'] & distancesAntecedents[2]['longe'] & distancesAntecedents[3]['longe'] & distancesAntecedents[4]['longe'] & distancesAntecedents[5]['longe'] & distancesAntecedents[6]['longe'] & distancesAntecedents[7]['longe'], vel_left['pos'])

avoid_obstacle_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])
avoid_obstacle = ctrl.ControlSystemSimulation(avoid_obstacle_ctrl)

positions = [[], []]
velocities_left = []
velocities_right = []

def fuzzy(dist, vel):
    i=0
    # print('dist:', dist)
    for d in dist:
        avoid_obstacle.input['distance_' + str(i)] = d
        i += 1
    avoid_obstacle.compute()
    return [avoid_obstacle.output['vel_left'], avoid_obstacle.output['vel_right']]

def get_info_to_plot(vel, it):
    if it % 200 == 0:
        position = robot.get_current_position()
        positions[0].append(position[0])
        positions[1].append(position[1])
        velocities_left.append(vel[0])
        velocities_right.append(vel[1])

robot = Robot()
i=0
while(robot.get_connection_status() != -1):
    us_distances = robot.read_ultrassonic_sensors()
    vel = fuzzy(us_distances[:8], 3) #Using only the 8 frontal sensors
    get_info_to_plot(vel, i)
    robot.set_left_velocity(vel[0])
    robot.set_right_velocity(vel[1])
    i += 1

plt.plot(positions[0], positions[1])
plt.plot(positions[0][0], positions[1][0], 'go')
plt.plot(positions[0][-1], positions[1][-1], 'rx')
plt.title('Posição do robô')
plt.show()

plt.plot(velocities_left)
plt.title('Velocidade motor esquerdo')
plt.show()

plt.plot(velocities_right)
plt.title('Velocidade motor direito')
plt.show()