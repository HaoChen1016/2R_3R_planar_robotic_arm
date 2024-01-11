"""
challenge.py
"""
import time
from typing import List, Tuple, Union
from robot import *
from visualizer import *
from obstacle import *
import numpy as np

class Runner:
    def __init__(
        self,
        robot: Robot,
        vis: Visualizer,
        obstacle:Obstacle
    ) -> None:
        self.robot = robot
        self.vis = vis
        self.obstacle = obstacle

    def run(self) -> None:
        running = True
        while running:
            # update robot kinematics
            self.robot.update_kinematics()

            # Check collisions
            ifCol = self.obstacle.check_collision()
            if ifCol == True:
                print("There is a collision!!!")
                #TODO
                #collision avoidance algorithm goes in here 
            else:
                pass

            # Check success
            success = self.check_success()

            # Update the display
            running = self.vis.update_display(self.robot, success)

            # sleep for Robot DT seconds, to force update rate
            time.sleep(self.robot.DT)


    def check_success(self) -> bool:
        """
        Check that robot's joint 2 is very close to the goal.
        Don't not use exact comparision, to be robust to floating point calculations.
        """
        world_goal = self.robot.world_goal
        joint2_pos = self.robot.world_pos2

        return np.allclose(joint2_pos, world_goal[0:2], atol=0.5)

    def cleanup(self) -> None:
        self.vis.cleanup()
