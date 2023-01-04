# -*- coding: utf-8 -*-*

import numpy as np
import math

from spg_overlay.utils.utils import normalize_angle


def followOptimizedTrajectory(self, the_lidar_sensor, the_trajectory, the_position, theta_d):
    """
    The Drone follows an optimized version of its trajectory to reach its first position.
    """
    [x_d, y_d] = the_position
    size = the_lidar_sensor.resolution
    values = the_lidar_sensor.get_sensor_values()
    g = 0
    theta_gd = 0
    goal_found = False
    dist_to_g = 0
    while not goal_found:
        [x_g, y_g] = the_trajectory[g]
        d_x = x_g - x_d
        d_y = y_g - y_d
        dist_to_g = math.sqrt(d_x**2 + d_y**2)
        if  dist_to_g > self.maxLidarRange:
            g += 1
        else:
            theta_g = math.atan2(d_y,d_x)
            theta_gd = normalize_angle(theta_d-theta_g)
            relative_direction_index = 0
            nb_of_rad_between_rays = 0.
            if size != 0:
                nb_of_rad_between_rays = (2*np.pi)/(size-1)
                relative_direction_index = int((size-1)/2) + round(-theta_gd/nb_of_rad_between_rays)
            goal_check = True
            for i in range(-12,13):
                direction_to_check = (relative_direction_index + i) % size
                if values[direction_to_check] < dist_to_g + 1:
                    goal_check = False
                    
            goal_found = goal_check
            if not goal_found:
                g += 1
        if g == len(the_trajectory):
            command_change_pov = {"forward": 0.0,
                            "lateral": 0.0,
                            "rotation": 0.01}
            return command_change_pov

    intensity = self.intensity
    force_intensity = 1
    long_force = math.cos(-theta_gd)
    lat_force = math.sin(-theta_gd)
    if dist_to_g < 10:
        force_intensity = intensity * math.atan(dist_to_g/10)*2/(np.pi)
    else:
        force_intensity = intensity/max(abs(long_force), abs(lat_force))
    long_force *= force_intensity
    lat_force *= force_intensity
    if long_force < -1:
        long_force = -1
    elif long_force > 1:
        long_force = 1
    if lat_force < -1:
        lat_force = -1
    elif lat_force > 1:
        lat_force = 1
    
    command_to_goal = {"forward": long_force,
                        "lateral": lat_force,
                        "rotation": 0}

    return command_to_goal