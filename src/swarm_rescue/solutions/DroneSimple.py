"""
Simple version of the drone used for Eval 1
....
"""

import random
from typing import Optional

import numpy as np
from numpy import pi

from spg_overlay.entities.drone_abstract import DroneAbstract
from spg_overlay.utils.misc_data import MiscData
from spg_overlay.utils.utils import normalize_angle

from solutions.sm.DroneStateMachine import DroneStateMachine


class DroneSimple(DroneAbstract):
    def __init__(self,
                 identifier: Optional[int] = None,
                 misc_data: Optional[MiscData] = None,
                 **kwargs):
        super().__init__(identifier=identifier,
                         misc_data=misc_data,
                         should_display_lidar=False,
                         **kwargs)
        self.counterStraight = 0
        self.counterExploring = 0
        self.angleStopTurning = random.uniform(-pi, pi)
        self.distStopStraight = random.uniform(10, 50)
        self.timeStopExploring = 300
        self.explorationTrajectory = []
        self.rescueTrajectory = []
        self.maxLidarRange = 290


        self.edgingDistance = 45
        self.counterAvoidingObstacle = 0
        self.intensity = 1
        self.timeChangeWall = 500
        self.counterFollowingWall = 0
        self.counterChangingWall = 0


        # COMMAND
        self.cmd_move = {"forward": 0,
                         "lateral": 0,
                         "rotation": 0}
        self.cmd_grasper = 0


        self.state = 'exploration'
        self.drone_sm = DroneStateMachine(self)

        print("State machine starts on exploration")


        # EKF
        self.est_pos = np.zeros(3)

        self.kalman_step_int = 0




    # State functions
    from solutions.sm.sm_functions import exploration_fct, rescue_victim_fct, return_base_fct, drop_victim_fct, repositionning_fct

    fct_dict = {'exploration' : exploration_fct,
                'rescue_victim' : rescue_victim_fct,
                'return_base' : return_base_fct,
                'drop_victim' : drop_victim_fct,
                'repositionning' : repositionning_fct}
    

    # EKF functions
    from solutions.ekf.ekf_functions import kalman_step, getKalmanStates



    def define_message_for_all(self):
        """
        Here, we don't need communication...
        """
        pass



    def control(self):
        """
        The Drone starts by exploring the map.
        After a certain amount of time, it goes back to its initial position.
        Once it has reached its initial position, it regains its previous exploration position and resumes exploration.
        """

        # print(len(self.explorationTrajectory))

        self.kalman_step()

        self.est_pos = self.getKalmanStates()

        # print(self.est_pos) #  -np.array([self.true_position()[0], self.true_position()[1], self.true_angle()])

        # print(f"Current state : {self.state}")
        self.fct_dict[self.state](self)

        # send command (TODO : passer par l'asserv)


        current_command = {"forward": self.cmd_move["forward"],
                           "lateral": self.cmd_move["lateral"],
                           "rotation": self.cmd_move["rotation"],
                           "grasper": self.cmd_grasper}

        return current_command
