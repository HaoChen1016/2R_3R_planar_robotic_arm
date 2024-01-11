"""
robot.py: this file defines 3R plannar robotic arm class
"""
from typing import List, Tuple, Union
import numpy as np
import math
class Robot:
    JOINT_LIMITS = [-6.28, 6.28]
    MAX_VELOCITY = 15
    MAX_ACCELERATION = 50
    DT = 0.033

    def __init__(self, param:dict) -> None:
        #parameters
        self.param         = param
        self.link_0: float = 75.  # pixels
        self.link_1: float = 50.  # pixels
        self.link_2: float = 25.  # pixels

        #  history states
        self.all_theta_0: List[float] = []
        self.all_theta_1: List[float] = []
        self.all_theta_2: List[float] = []
        self.all_world_pos2 = np.empty((0, 2))

        # states
        self.local_pos0    = () #local position of joint0(first joint)
        self.local_pos1    = () #local position of joint1
        self.local_pos2    = () #local position of joint2
        self.world_pos0    = () #world position of joint0(first joint)
        self.world_pos1    = () #world position of joint1
        self.world_pos2    = () #world position of joint2

        self.states        = [0.,0.,0.]
        self.desiredStates = []
        self.world_origin  = param["world_origin"]
        self.world_goal    = [622, 598, 3.075158962858593]
        x,y    = self.world_to_local(self.world_goal[0:2])
        self.local_goal  = [x,y,self.world_goal[2]]
        # self.world_goal    = []
        # self.local_goal    = []
        
 
    def states_get(self) -> Tuple[float, float,float]:
        '''
        get states which includes theta_0, theta_1, theta_2
        '''
        return self.states 

 
    def states_set(self, desiredStates: Tuple[float, float,float]) -> None:
        '''
        set states which includes theta_0, theta_1, theta_2
        '''
        self.all_theta_0.append(desiredStates[0])
        self.all_theta_1.append(desiredStates[1])
        self.all_theta_2.append(desiredStates[2])

        self.states = desiredStates

        # Check limits
        # assert self.check_angle_limits(value), \
        #     f'Joint 0 value {value} exceeds joint limits'
        # assert self.max_velocity(self.all_theta_0) < self.MAX_VELOCITY, \
        #     f'Joint 0 Velocity {self.max_velocity(self.all_theta_0)} exceeds velocity limit'
        # assert self.max_acceleration(self.all_theta_0) < self.MAX_ACCELERATION, \
        #     f'Joint 0 Accel {self.max_acceleration(self.all_theta_0)} exceeds acceleration limit'

    def update_kinematics(self) -> None:
        '''
        update robot kinematics
        '''
        # update theta
        self.controller()
        # update position
        self.f_kinematics_3R()


    def controller(self)->None:
        '''
        robot controller
        '''
        self.desiredStates = self.i_kinematics_3R()

        #Simple P controller
        theta_0_error = self.desiredStates[0] - self.states[0]
        theta_1_error = self.desiredStates[1] - self.states[1]
        theta_2_error = self.desiredStates[2] - self.states[2]

        self.states[0] += theta_0_error / 10
        self.states[1] += theta_1_error / 10
        self.states[2] += theta_2_error / 10

        
    def f_kinematics_3R(self) -> None:
        '''
        forward kinematics update.
        input:  
        output: 
        ''' 
        #states
        theta_0 = self.states[0]
        theta_1 = self.states[1]
        theta_2 = self.states[2]

        #param
        link_0 = self.link_0
        link_1 = self.link_1
        link_2 = self.link_2

        #forward kinematics
        local_pos0  = (link_0 * np.cos(theta_0),link_0 * np.sin(theta_0))
        
        x = link_0 * np.cos(theta_0) + link_1 * np.cos(theta_0 + theta_1)
        y = link_0 * np.sin(theta_0) + link_1 * np.sin(theta_0 + theta_1)
        local_pos1  = (x,y)

        x = link_0 * np.cos(theta_0) + link_1 * np.cos(theta_0 + theta_1) + link_2 * np.cos(theta_0 + theta_1 + theta_2)
        y = link_0 * np.sin(theta_0) + link_1 * np.sin(theta_0 + theta_1) + link_2 * np.sin(theta_0 + theta_1 + theta_2)
        local_pos2  = (x,y)

        self.local_pos0 = local_pos0
        self.local_pos1 = local_pos1
        self.local_pos2 = local_pos2
        self.world_pos0 = self.local_to_world(local_pos0)
        self.world_pos1 = self.local_to_world(local_pos1)
        self.world_pos2 = self.local_to_world(local_pos2)

        worldPos2_array = np.array([self.world_pos2[0], self.world_pos2[1]])
        self.all_world_pos2 = np.vstack((self.all_world_pos2,worldPos2_array))


    def i_kinematics_2R(self) -> Tuple[float, float]:
        '''
        inverse kinematics update.Compute the joint angles from the position of the end of the links
        ''' 
        x = self.local_goal[0]
        y = self.local_goal[1]
        #param
        link_0 = self.link_0
        link_1 = self.link_1
        link_2 = self.link_2

        theta_1 = np.arccos((x ** 2 + y ** 2 - link_0 ** 2 - link_1 ** 2)/ (2 * link_0 * link_1))
        theta_0 = np.arctan2(y, x) - np.arctan((link_0 * np.sin(theta_1)) /
                      (link_0 + link_1 * np.cos(theta_1)))

        return theta_0, theta_1
    
    def i_kinematics_3R(self) -> Tuple[float, float, float]:
        """
        inverse kinematics update. Compute the joint angles from the position of the end of the links
        """
        xe = self.local_goal[0]
        ye = self.local_goal[1]
        gamma = self.local_goal[2]

        #param
        link_0 = self.link_0
        link_1 = self.link_1
        link_2 = self.link_2

        # position P2
        x2 = xe-(link_2*np.cos(gamma))
        y2 = ye-(link_2*np.sin(gamma))
        C  = np.sqrt(x2**2 + y2**2)
        # print(link_0 + link_1)
        # print(C)
        if link_0 + link_1 > C:
            alpha = np.arccos((link_0**2 + link_1**2 - C**2 )/(2*link_0*link_1))
            beta  = np.arcsin(link_1*np.sin(alpha)/C)
            # beta  = np.arccos((C**2 + self.link_1**2 - self.link_2**2)/(2*self.link_1*C))

            #joint angles elbow-down
            theta_0 = np.arctan(y2/x2)-beta
            theta_1 = math.pi-alpha
            theta_2 = gamma - theta_0 -theta_1
            # print(theta_0)

            #joint angles elbow-up
            # theta_0 = np.arctan(y3,x3)+beta
            # theta_1 = -(180-alpha)
            # theta_2 = gamma - theta_0 -theta_1
        else:
            print('End-effecter is outside the workspace.')
        return theta_0, theta_1, theta_2
    
    def local_to_world(
        self, local_pos: Tuple[Union[int, float], Union[int, float]]) -> Tuple[int, int]:
        """
        Convert a point from the robot coordinate system(local frame) to the display coordinate system(world frame)
        """
        local_x, local_y   = local_pos
        offset_x, offset_y = self.world_origin
        return int(offset_x + local_x), int(offset_y + local_y)
    
    def world_to_local(
        self, world_pos: Tuple[Union[int, float], Union[int, float]]) -> Tuple[int, int]:
        """
        Convert a point from the display coordinate system(world frame)  to the robot coordinate system(local frame)
        """
        world_x, world_y   = world_pos
        offset_x, offset_y = self.world_origin
        return int(world_x - offset_x), int(world_y - offset_y)

    def generate_random_goal(self) -> None:
        """
        Generate a random goal that is reachable by the robot arm
        """
        min_radius = self.min_reachable_radius()
        max_radius = self.max_reachable_radius()
        # Ensure theta is not 0
        theta = (np.random.random() + np.finfo(float).eps) * 2 * np.pi
        # Ensure point is reachable
        r = np.random.uniform(low=min_radius, high=max_radius)

        goal_angle   = np.random.uniform(low=0, high=2 * np.pi)
        local_goal_x = int(r * np.cos(theta))
        local_goal_y = int(r * np.sin(theta))
        world_goal   = self.local_to_world((local_goal_x,local_goal_y))
        world_goal_x = world_goal[0]
        world_goal_y = world_goal[1]
        self.world_goal = [world_goal_x,world_goal_y,goal_angle]
        self.local_goal = [local_goal_x,local_goal_y,goal_angle]

    def min_reachable_radius(self) -> float:
        return max(self.link_0 - self.link_1 - self.link_2, 0)

    def max_reachable_radius(self) -> float:
        return self.link_0 + self.link_1 + self.link_2
    
    @classmethod
    def check_angle_limits(cls, theta: float) -> bool:
        return cls.JOINT_LIMITS[0] < theta < cls.JOINT_LIMITS[1]

    @classmethod
    def max_velocity(cls, all_theta: List[float]) -> float:
        return float(max(abs(np.diff(all_theta) / cls.DT), default=0.))

    @classmethod
    def max_acceleration(cls, all_theta: List[float]) -> float:
        return float(max(abs(np.diff(np.diff(all_theta)) / cls.DT / cls.DT), default=0.))


