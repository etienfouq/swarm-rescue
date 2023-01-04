# -*- coding: utf-8 -*-*


import random
from time import sleep

import numpy as np
from numpy import pi

from spg_overlay.utils.utils import normalize_angle

from solutions.pathfinder.trajectory import followOptimizedTrajectory


DISTANCE_TO_DEST = 5

def command_go_straight(self):
    return {"forward": self.intensity,
            "lateral": 0.0,
            "rotation": 0.0}

def command_turn_left(self):
    return {"forward": 0.0,
            "lateral": self.intensity,
            "rotation": self.intensity}

def command_wait(self):
    return {"forward": 0.0,
            "lateral": 0.0,
            "rotation": 0.0}



def exploration_fct(self):
    """
    The Drone explores the map by moving along the wall on its right
    """

    semantic_data = self.semantic().get_sensor_values()

    # transition : found wounded person
    # TODO : more subtle
    for i in range(len(semantic_data)):
        if str(semantic_data[i].entity_type) == "TypeEntity.WOUNDED_PERSON":
            return self.drone_sm.victim_found()
    

    the_position = self.est_pos[:2]
    if the_position[0] != 0 or the_position[1] != 0:  # added because the ekf gives a (0,0) position at start
        self.explorationTrajectory.append(the_position)



    if self.counterFollowingWall == self.timeChangeWall:
        self.counterChangingWall = 25
        self.counterFollowingWall = 0


    values = self.lidar().get_sensor_values()
    size = self.lidar().resolution
    distances = [0.] * 48
    for i in range(48):
        dist = 0.
        for j in range(int(size/48)):
            index = int(7 * size/4 - (size/48) * (i + 1)  + j) % size
            dist += values[index]

        distances[i] = dist/(size/48)


    edgingDistance = self.edgingDistance

    obstacle_in_front = distances[11] < edgingDistance or distances[12] < edgingDistance 

    if self.counterChangingWall > 16:
        self.counterChangingWall -= 1
        self.cmd_move = command_turn_left(self)
        return

    elif self.counterChangingWall > 0 and not obstacle_in_front:
        self.counterChangingWall -= 1
        self.cmd_move = command_go_straight(self)
        return

    elif self.counterChangingWall > 0 and obstacle_in_front:
        self.counterChangingWall -=1
        self.cmd_move = command_wait(self)
        return


    if (obstacle_in_front) and self.counterAvoidingObstacle == 0:
        self.counterAvoidingObstacle = 3

    if self.counterAvoidingObstacle > 0:
        self.counterAvoidingObstacle -= 1
        self.cmd_move = command_turn_left(self)
        return
    
    
    for i in range(24, -24, -1):
        obstacle_index = (i + 48) % 48
        if distances[obstacle_index] < edgingDistance:
            direction_found = True
            for j in range (4, 48):
                for k in range (-3, 3):
                    k_th_direction_index = (obstacle_index + 48 - j + k) % 48
                    if distances[k_th_direction_index] <= edgingDistance:
                        direction_found = False
                if direction_found:
                    direction_index = (obstacle_index + 48 - j ) % 48
                    direction_angle = (180 - (direction_index * (size/24) + size/96))*np.pi/180 + 0.1
                        
                    forward_force = self.intensity * np.sin(direction_angle)
                    lateral_force = - self.intensity * np.cos(direction_angle)
                    rotation = - self.intensity * np.cos(direction_angle)

                    self.counterFollowingWall += 1
                    self.cmd_move = {"forward": forward_force,
                                "lateral": lateral_force,
                                "rotation": rotation}
                    return



def rescue_victim_fct(self):
    """
    Appelée pendant la phase de grasp de la victime
    Transitionne lorsque la victime a été grasp (elle sort du champ de vision)
    """

    semantic_data = self.semantic().get_sensor_values()

    the_position = self.est_pos[:2]

    self.explorationTrajectory.append(the_position)

    wounded_people = []
    for i in range(len(semantic_data)):

        if str(semantic_data[i].entity_type) == "TypeEntity.WOUNDED_PERSON":

            wounded_people.append((semantic_data[i].distance, semantic_data[i].angle))

    # on a la liste des détections de victimes
            
    if len(wounded_people) == 0:  # TODO : cas bizarre, on n'a plus la victime en vue (repasser à l'exploration ?)
        print("ERROR : LOST VICTIM")
        return

    position_wounded = min(wounded_people)

    command = {"forward": 0.5,
               "lateral": 0.0,
               "rotation": position_wounded[1]/np.pi}
    
    self.cmd_move = command

    if position_wounded[0] < 25:  # px TODO : parameter
        self.cmd_grasper = 1
        print("Victim grasped")
        return self.drone_sm.victim_grasped()  # transition

    else:
        self.cmd_grasper = 0
        return

    



    
def return_base_fct(self):


    # the_position = self.est_pos[:2]

    the_position = self.true_position()


    the_exploration_trajectory = self.explorationTrajectory

    if abs(the_position[0] - the_exploration_trajectory[1][0]) < DISTANCE_TO_DEST and abs(the_position[1] - the_exploration_trajectory[1][1]) < DISTANCE_TO_DEST:
        self.explorationTrajectory = []
        return self.drone_sm.oriented_to_base()
    

    else :
        the_lidar_sensor = self.lidar()
        theta_d = self.true_angle()
        self.rescueTrajectory.append(the_position)
        self.cmd_move = followOptimizedTrajectory(self, the_lidar_sensor, the_exploration_trajectory, the_position, theta_d)
        return


    
def drop_victim_fct(self):
    """
    Appelée pendant la phase de drop de la victime
    Transitionne lorsque la victime a été placée dans la base
    """


    semantic_data = self.semantic().get_sensor_values()

    the_position = self.est_pos[:2]

    self.rescueTrajectory.append(the_position)

    rescue_points = []
    for i in range(len(semantic_data)):

        if str(semantic_data[i].entity_type) == "TypeEntity.RESCUE_CENTER":

            rescue_points.append((semantic_data[i].distance, semantic_data[i].angle))

    # on a la liste des détections de victimes
            
    if len(rescue_points) == 0:  # TODO : cas bizarre, on n'a plus la victime en vue (repasser à l'exploration ?)
        print("ERROR : LOST RESCUE CENTER")
        return

    position_wounded = min(rescue_points)

    command = {"forward": 0.5,
               "lateral": 0.0,
               "rotation": position_wounded[1]/np.pi}
    
    self.cmd_move = command

    if len(self.base.grasper.grasped_entities) == 0:
        self.cmd_grasper = 0
        print("Victim dropped at rescue center")
        return self.drone_sm.victim_dropped()  # transition

    else:
        self.cmd_grasper = 1
        return



    
def repositionning_fct(self):

    the_position = self.est_pos[:2]

    the_rescue_trajectory = self.rescueTrajectory

    if abs(the_position[0] - the_rescue_trajectory[1][0]) < DISTANCE_TO_DEST and abs(the_position[1] - the_rescue_trajectory[1][1]) < DISTANCE_TO_DEST:

        self.explorationTrajectory = list(reversed(the_rescue_trajectory))
        self.rescueTrajectory = []
        self.counterExploring = 1
        self.counterStraight = 0

        return self.drone_sm.back_to_exploration()
    
    else:
        the_lidar_sensor = self.lidar()
        theta_d = self.true_angle()
        self.cmd_move = followOptimizedTrajectory(self, the_lidar_sensor, the_rescue_trajectory, the_position, theta_d)
        return


