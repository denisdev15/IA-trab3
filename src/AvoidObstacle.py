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
vel_left = ctrl.Consequent(np.arange(0, 5, 1), 'vel_left')
vel_right = ctrl.Consequent(np.arange(0, 5, 1), 'vel_right')

for i in range(8):
    distancesAntecedents[i]["perto"] = fuzz.trapmf(distancesAntecedents[i].universe, [0, 0, 0.2, 1])
    distancesAntecedents[i]["longe"] = fuzz.trapmf(distancesAntecedents[i].universe, [0.2, 1, 6, 6])
    # distancesAntecedents[i]["perto"].view()
    # # distancesAntecedents[i]["longe"].view()
    # plt.show()

vel_left['on'] = fuzz.trimf(vel_left.universe, [0, 4, 4])
vel_right['on'] = fuzz.trimf(vel_right.universe, [0, 4, 4])

vel_left.view()
vel_right.view()

plt.show()

# rule1 = ctrl.Rule(distancesAntecedents[0]['perto'] | distancesAntecedents[1]['perto'] | distancesAntecedents[2]['perto'] | distancesAntecedents[3]['perto'], vel_right['off'])
# rule2 = ctrl.Rule(distancesAntecedents[4]['perto'] | distancesAntecedents[5]['perto'] | distancesAntecedents[6]['perto'] | distancesAntecedents[7]['perto'], vel_left['off'])
# rule3 = ctrl.Rule(distancesAntecedents[0]['longe'] | distancesAntecedents[1]['longe'] | distancesAntecedents[2]['longe'] | distancesAntecedents[3]['longe'] | distancesAntecedents[4]['longe'] | distancesAntecedents[5]['longe'] | distancesAntecedents[6]['longe'] | distancesAntecedents[7]['longe'], vel_left['on'])
# rule4 = ctrl.Rule(distancesAntecedents[0]['longe'] | distancesAntecedents[1]['longe'] | distancesAntecedents[2]['longe'] | distancesAntecedents[3]['longe'] | distancesAntecedents[4]['longe'] | distancesAntecedents[5]['longe'] | distancesAntecedents[6]['longe'] | distancesAntecedents[7]['longe'], vel_left['off'])

# avoid_obstacle_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4])

# avoid_obstacle = ctrl.ControlSystemSimulation(avoid_obstacle_ctrl)

# for i in range(4):
#     avoid_obstacle.input['distance_' + str(i)] = 0.6
# for i in range(4):
#     avoid_obstacle.input['distance_' + str(i+4)] = 4

# print(avoid_obstacle.output)
# print(avoid_obstacle.output['vel_right'])

# # Crunch the numbers
# tipping.compute()



# def fuzzy(dist, vel):
#     # TODO fuzzy logic
#     return [0, 0]

# robot = Robot()
# while(robot.get_connection_status() != -1):
#     us_distances = robot.read_ultrassonic_sensors()
#     vel = fuzzy(us_distances[:8], 3) #Using only the 8 frontal sensors
#     robot.set_left_velocity(vel[0])
#     robot.set_right_velocity(vel[1])
