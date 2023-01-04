# -*- coding: utf-8 -*-

import numpy as np
import math

# from spg_overlay.entities.drone_abstract import DroneAbstract

from spg_overlay.utils.utils import deg2rad



def kalman_step(self):
    self.kalman_step_int += 1

    if self.kalman_step_int == 1:
        return

    if not hasattr(self, "kalman_record"):
        kalman_init(self)

    kalman_predict(self)

    if not self.gps_is_disabled():
        kalman_update_gps(self)
    if not self.compass_is_disabled():
        kalman_update_comp(self)

    # pos_error = 100*(np.sqrt(self.kalman_record[-1][0] ** 2 + self.kalman_record[-1][1] ** 2) -
    #              np.sqrt(self.true_position()[0]**2 + self.true_position()[0])) / \
    #             np.sqrt(self.true_position()[0]**2 + self.true_position()[0])
    # the_error = 100*(self.kalman_record[-1][2] - self.true_angle()) / self.true_angle()
    # if self.kalman_step_int % 15 == 0:
    #     print(self.true_position()[1], self.kalman_record[-1][1])




def kalman_init(self):


    self.kalman_Q = 1000*np.diag(
        [
            1,
            1,
            1,
            1
        ]
    )
    self.kalman_R_gps = 0.0001*(
            np.diag(
                [
                    1,
                    1,
                ]
            ) ** 2
    )
    self.kalman_R_comp = 0.0001*(
            np.diag(
                [
                    1,
                    1
                ]
            ) ** 2
    )

    self.initial_pos = self.measured_gps_position()
    self.initial_att = self.measured_compass_angle()
    self.kalman_record = [np.array([self.initial_pos[0], self.initial_pos[1], math.sin(self.initial_att[0]),
                                    math.cos(self.initial_att[0])])]
    self.kalman_P = self.kalman_Q


def kalman_predict(self):
    if self.kalman_record[-1][0] > 10**5 : self.kalman_record.append(np.array([self.initial_pos[0], self.initial_pos[1],
                                                                               math.sin(self.initial_att[0]),
                                                                               math.cos(self.initial_att[0])]))
    if not self.gps_is_disabled():
        if np.linalg.linalg.norm(self.kalman_P) > 1:
            self.kalman_P = self.kalman_Q
        # Prediction
        Xnp1 = self.kalman_record[-1]

        self.kalman_record.append(Xnp1)
        F = np.array([[1, 0, 0, 0],
                      [0, 1, 0, 0],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]])

        Pnp1 = F @ self.kalman_P @ np.transpose(F)
        self.kalman_P = Pnp1
    else:
        # System states
        X = self.kalman_record[-1][0]
        Y = self.kalman_record[-1][1]
        the = math.atan2(self.kalman_record[-1][2], self.kalman_record[-1][3])

        # Sensor values
        d_odo, alpha_odo, theta_odo = self.odometer_values()

        # Prediction
        Xnp1 = X + d_odo * np.cos(the + alpha_odo)
        Ynp1 = Y + d_odo * np.sin(the + alpha_odo)
        thenp1 = the + theta_odo
        Xnp1 =np.array([Xnp1, Ynp1, math.sin(thenp1), math.cos(thenp1)])

        self.kalman_record.append(Xnp1)
        # F = np.array([[1, 0, 1, 1],
        #               [0, 1, 1, 1],
        #               [0, 0, 1, 1],
        #               [0, 0, 1, 1]])
        F = np.array([[1, 0, -math.sin(the + alpha_odo)/math.cos(the), math.sin(the + alpha_odo)/math.sin(the)],
                      [0, 1, math.cos(the + alpha_odo)/math.cos(the), -math.cos(the + alpha_odo)/math.sin(the)],
                      [0, 0, math.cos(the + theta_odo)/math.cos(the), -math.cos(the + theta_odo)/math.sin(the)],
                      [0, 0, -math.sin(the + theta_odo)/math.cos(the), math.sin(the + theta_odo)/math.sin(the)]])

        Pnp1 = F @ self.kalman_P @ np.transpose(F)
        if np.linalg.linalg.norm(Pnp1) > 10**5:
            Pnp1 = Pnp1 * 0.0001
        self.kalman_P = Pnp1

def kalman_update_gps(self):
    if np.linalg.linalg.norm(self.kalman_P) > 1:
        self.kalman_P = self.kalman_Q
    Y = np.array(self.measured_gps_position())
    # print(self.measured_gps_position())
    Y_hat = np.array([self.kalman_record[-1][0], self.kalman_record[-1][1]])
    H = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0]])

    S = H @ self.kalman_P @ np.transpose(H) + self.kalman_R_gps
    K = self.kalman_P @ np.transpose(H) @ np.linalg.inv(S)

    self.kalman_record.append(np.array(self.kalman_record[-1] + 1 * K @ (Y - Y_hat)))
    self.kalman_P = self.kalman_P - K @ S @ np.transpose(K)


def kalman_update_comp(self):
    if np.linalg.linalg.norm(self.kalman_P) > 1:
        self.kalman_P = self.kalman_Q
    Y = np.array([math.sin(self.measured_compass_angle()), math.cos(self.measured_compass_angle())])
    Y_hat = np.array([self.kalman_record[-1][2], self.kalman_record[-1][3]])
    H = np.array([[0, 0, 1, 0],
                  [0, 0, 0, 1]])

    S = H @ self.kalman_P @ np.transpose(H) + self.kalman_R_comp #(1x1)
    K = self.kalman_P @ np.transpose(H) @ np.linalg.inv(S) # (3x1)
    Xnp1 = np.array(self.kalman_record[-1] + 1 * K @ (Y - Y_hat))

    self.kalman_record.append(Xnp1)
    self.kalman_P = self.kalman_P - K @ S @ np.transpose(K)


def getKalmanStates(self):
    if self.kalman_step_int == 1:
        return np.array([0, 0, 0])

    if hasattr(self, 'kalman_record'):
        res = np.array([self.kalman_record[-1][0], self.kalman_record[-1][1], math.atan2(self.kalman_record[-1][2],
                                                                                             self.kalman_record[-1][3])])
        return res