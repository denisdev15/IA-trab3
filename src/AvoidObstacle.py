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
    # distancesAntecedents[i]["muito_perto"] = fuzz.trapmf(distancesAntecedents[i].universe, [0, 0, 0.4, 0.8])
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
# rule7 = ctrl.Rule(distancesAntecedents[4]['muito_perto'] & distancesAntecedents[5]['muito_perto'], vel_left['neg'])
# rule8 = ctrl.Rule(distancesAntecedents[4]['muito_perto'] & distancesAntecedents[5]['muito_perto'], vel_right['neg'])

avoid_obstacle_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])
avoid_obstacle = ctrl.ControlSystemSimulation(avoid_obstacle_ctrl)


def fuzzy(dist, vel):
    i=0
    print('dist:', dist)
    for d in dist:
        avoid_obstacle.input['distance_' + str(i)] = d
        i += 1
    avoid_obstacle.compute()
    return [avoid_obstacle.output['vel_left'], avoid_obstacle.output['vel_right']]

robot = Robot()
while(robot.get_connection_status() != -1):
    us_distances = robot.read_ultrassonic_sensors()
    vel = fuzzy(us_distances[:8], 3) #Using only the 8 frontal sensors
    print(vel)
    robot.set_left_velocity(vel[0])
    robot.set_right_velocity(vel[1])
