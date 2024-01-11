"""
challenge.py
"""
import time
from typing import List, Tuple, Union
from robot import *

import numpy as np
import pygame

# class Obstacle:
#     def __init__(self) -> None:
#         center = [300,250]
#         radius = 10
#         # pygame.draw.line(self.screen, self.BLACK, p1, p2, 10)

#     def check_collision(self, robot: Robot) -> Robot:
#         """
#         Check collision
#         """
#         #return robot


class Obstacle:
    def __init__(self,robot: Robot) -> None:
        # center = [300,250]
        # radius = 10
        # pygame.draw.line(self.screen, self.BLACK, p1, p2, 10)

        # define the static wall
        # wall_p1 = [700,650]
        # wall_p2 = [700,550]
        # wall_p3 = [710,550]
        # wall_p4 = [710,650]

        wall_p1 = [650,650]
        wall_p2 = [650,550]
        wall_p3 = [660,550]
        wall_p4 = [660,650]
        

        num_values = int(abs(wall_p1[1] - wall_p2[1]) / 1) + 1
        x_values   = np.linspace(wall_p1[0], wall_p2[0], num=num_values)
        y_values   = np.linspace(wall_p1[1], wall_p2[1], num=num_values)
        wall_12    = np.vstack((x_values, y_values)).T

        num_values = int(abs(wall_p2[0] - wall_p3[0]) / 1) + 1
        x_values   = np.linspace(wall_p2[0], wall_p3[0], num=num_values)
        y_values   = np.linspace(wall_p2[1], wall_p3[1], num=num_values)
        wall_23    = np.vstack((x_values, y_values)).T

        num_values = int(abs(wall_p3[1] - wall_p4[1]) / 1) + 1
        x_values   = np.linspace(wall_p3[0], wall_p4[0], num=num_values)
        y_values   = np.linspace(wall_p3[1], wall_p4[1], num=num_values)
        wall_34    = np.vstack((x_values, y_values)).T

        num_values = int(abs(wall_p4[0] - wall_p1[0]) / 1) + 1
        x_values   = np.linspace(wall_p4[0], wall_p1[0], num=num_values)
        y_values   = np.linspace(wall_p4[1], wall_p1[1], num=num_values)
        wall_41    = np.vstack((x_values, y_values)).T

        self.wall       = np.vstack((wall_12, wall_23, wall_34,wall_41))
        self.wall_p1 = wall_p1
        self.wall_p2 = wall_p2
        self.wall_p3 = wall_p3
        self.wall_p4 = wall_p4
        self.robot   = robot



    def check_collision(self) -> bool:
        """
        Check collision
        """
        #Find the farthest point in the robot from the origin
        world_origin = self.robot.world_origin
        world_pos0   = self.robot.world_pos0
        world_pos1   = self.robot.world_pos1
        world_pos2   = self.robot.world_pos2

        #
        j0_dis = np.linalg.norm(np.array(world_pos0) - np.array(world_origin))
        j1_dis = np.linalg.norm(np.array(world_pos1) - np.array(world_origin))
        j2_dis = np.linalg.norm(np.array(world_pos2) - np.array(world_origin))

        dis_robot_wall = []
        if j0_dis > j1_dis and j1_dis > j2_dis:
            for i in range(len(self.wall)):
                dis = np.linalg.norm(world_pos0 - self.wall[i][:])
                dis_robot_wall.append(dis)
        elif j1_dis > j0_dis and j1_dis > j2_dis:
            for i in range(len(self.wall)):
                dis = np.linalg.norm(world_pos1 - self.wall[i][:])
                dis_robot_wall.append(dis)
        elif j2_dis > j0_dis and j2_dis > j1_dis:
            for i in range(len(self.wall)):
                dis = np.linalg.norm(world_pos2 - self.wall[i][:])
                dis_robot_wall.append(dis)
        # print(dis_robot_wall)
        dis_tolerance =10
        if min(dis_robot_wall) < dis_tolerance:
            return True
        elif min(dis_robot_wall) > dis_tolerance:
            return False