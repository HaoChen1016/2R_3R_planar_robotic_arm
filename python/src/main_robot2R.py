"""
challenge.py
"""
import time
from typing import List, Tuple, Union

import numpy as np
import pygame
import matplotlib.pyplot as plt


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

        #  history states
        self.all_theta_0: List[float] = []
        self.all_theta_1: List[float] = []
        self.all_world_pos1 = np.empty((0, 2))

        # states
        self.local_pos0    = () #local position of joint0(first joint)
        self.local_pos1    = () #local position of joint1
        self.world_pos0    = () #world position of joint0(first joint)
        self.world_pos1    = () #world position of joint1

        self.states        = [0.,0.]
        self.desiredStates = []
        self.world_origin  = param["world_origin"]
        # self.world_goal    = [670, 670]
        # x,y    = self.world_to_local(self.world_goal)
        # self.local_goal  = [x,y]
        self.world_goal    = []
        self.local_goal    = []
        self.world_start   = []
        self.world_goal    = []
        
 
    def states_get(self) -> Tuple[float, float]:
        '''
        get states which includes theta_0, theta_1
        '''
        return self.states 

 
    def states_set(self, desiredStates: Tuple[float, float]) -> None:
        '''
        set states which includes theta_0, theta_1
        '''
        self.all_theta_0.append(desiredStates[0])
        self.all_theta_1.append(desiredStates[1])

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
        self.f_kinematics_2R()


    def controller(self)->None:
        '''
        robot controller
        '''
        self.desiredStates = self.i_kinematics_2R()

        #Simple P controller
        theta_0_error = self.desiredStates[0] - self.states[0]
        theta_1_error = self.desiredStates[1] - self.states[1]

        self.states[0] += theta_0_error / 5
        self.states[1] += theta_1_error / 5

        
    def f_kinematics_2R(self) -> None:
        '''
        forward kinematics update.
        input:  
        output: 
        ''' 
        #states
        theta_0 = self.states[0]
        theta_1 = self.states[1]

        #param
        link_0 = self.link_0
        link_1 = self.link_1

        #forward kinematics
        local_pos0  = (link_0 * np.cos(theta_0),link_0 * np.sin(theta_0))
        
        x = link_0 * np.cos(theta_0) + link_1 * np.cos(theta_0 + theta_1)
        y = link_0 * np.sin(theta_0) + link_1 * np.sin(theta_0 + theta_1)
        local_pos1  = (x,y)

        self.local_pos0 = local_pos0
        self.local_pos1 = local_pos1
        self.world_pos0 = self.local_to_world(local_pos0)
        self.world_pos1 = self.local_to_world(local_pos1)

        worldPos1_array = np.array([self.world_pos1[0], self.world_pos1[1]])
        self.all_world_pos1 = np.vstack((self.all_world_pos1,worldPos1_array))


    def i_kinematics_2R(self) -> Tuple[float, float]:
        '''
        inverse kinematics update.Compute the joint angles from the position of the end of the links
        ''' 
        x = self.local_goal[0]
        y = self.local_goal[1]
        #param
        link_0 = self.link_0
        link_1 = self.link_1

        theta_1 = np.arccos((x ** 2 + y ** 2 - link_0 ** 2 - link_1 ** 2)/ (2 * link_0 * link_1))
        theta_0 = np.arctan2(y, x) - np.arctan((link_1 * np.sin(theta_1)) /
                      (link_0 + link_1 * np.cos(theta_1)))

        return theta_0, theta_1
    
    def local_to_world(
        self, local_pos: Tuple[Union[int, float], Union[int, float]]) -> Tuple[int, int]:
        """
        Convert a point from the robot coordinate system(local frame) to the display coordinate system(world frame)
        """
        local_x, local_y   = local_pos
        offset_x, offset_y = self.world_origin
        # print('local_pos',local_pos)
        # print('world_origin',self.world_origin)
        return int(offset_x + local_x), int(offset_y + local_y)
    
    def world_to_local(
        self, world_pos: Tuple[Union[int, float], Union[int, float]]) -> Tuple[int, int]:
        """
        Convert a point from the display coordinate system(world frame)  to the robot coordinate system(local frame)
        """
        world_x, world_y   = world_pos
        offset_x, offset_y = self.world_origin
        return int(world_x - offset_x), int(world_y - offset_y)

    def generate_random_point(self) -> None:
        """
        Generate a random goal that is reachable by the robot arm
        """
        min_radius = self.min_reachable_radius()
        max_radius = self.max_reachable_radius()
        # Ensure theta is not 0
        theta = (np.random.random() + np.finfo(float).eps) * 2 * np.pi
        # Ensure point is reachable
        r = np.random.uniform(low=min_radius, high=max_radius)

        local_goal_x = int(r * np.cos(theta))
        local_goal_y = int(r * np.sin(theta))
        world_goal   = self.local_to_world((local_goal_x,local_goal_y))
        world_goal_x = world_goal[0]
        world_goal_y = world_goal[1]
        self.world_goal = [world_goal_x,world_goal_y]
        self.local_goal = [local_goal_x,local_goal_y]

        # for generate start point
        theta0 = (np.random.random() + np.finfo(float).eps) * 2 * np.pi
        theta1 = (np.random.random() + np.finfo(float).eps) * 2 * np.pi
        self.states = [theta0,theta1]
        self.f_kinematics_2R()
        self.world_start = self.world_pos1

    def min_reachable_radius(self) -> float:
        return max(self.link_0 - self.link_1, 0)

    def max_reachable_radius(self) -> float:
        return self.link_0 + self.link_1
    
    @classmethod
    def check_angle_limits(cls, theta: float) -> bool:
        return cls.JOINT_LIMITS[0] < theta < cls.JOINT_LIMITS[1]

    @classmethod
    def max_velocity(cls, all_theta: List[float]) -> float:
        return float(max(abs(np.diff(all_theta) / cls.DT), default=0.))

    @classmethod
    def max_acceleration(cls, all_theta: List[float]) -> float:
        return float(max(abs(np.diff(np.diff(all_theta)) / cls.DT / cls.DT), default=0.))

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

        #
        j0_dis = np.linalg.norm(np.array(world_pos0) - np.array(world_origin))
        j1_dis = np.linalg.norm(np.array(world_pos1) - np.array(world_origin))

        dis_robot_wall = []
        if j0_dis > j1_dis:
            for i in range(len(self.wall)):
                dis = np.linalg.norm(world_pos0 - self.wall[i][:])
                dis_robot_wall.append(dis)
        elif j0_dis < j1_dis:
            for i in range(len(self.wall)):
                dis = np.linalg.norm(world_pos1 - self.wall[i][:])
                dis_robot_wall.append(dis)
        # print(dis_robot_wall)
        dis_tolerance =10
        if min(dis_robot_wall) < dis_tolerance:
            return True
        elif min(dis_robot_wall) > dis_tolerance:
            return False



class  Visualizer:
    BLACK: Tuple[int, int, int] = (0, 0, 0)
    RED:   Tuple[int, int, int] = (255, 0, 0)
    WHITE: Tuple[int, int, int] = (255, 255, 255)
    BLUE:  Tuple[int, int, int] = (0, 0, 205)
    GREEN: Tuple[int, int, int] = (0, 255, 0)

    def __init__(self,param:dict, robot:Robot, obstacle:Obstacle) -> None:
        """
        Note: while the Robot and World have the origin in the center of the
        visualization, rendering places (0, 0) in the top left corner.
        """
        height = param["height"]
        width  = param["width"]
        self.robot = robot
        self.obstacle = obstacle

        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('Gherkin Challenge')
        self.font = pygame.font.SysFont('freesansbolf.tff', 30)

        small_window_size = (200, 100)
        self.small_screen = pygame.Surface(small_window_size)
        

    def display_goal_start(self) -> None:
        """
        Display the goal and start position
        """
        pygame.draw.circle(self.screen, self.RED, self.robot.world_goal[0:2], 6)
        pygame.draw.circle(self.screen, self.GREEN, self.robot.world_start, 6)
    
    def display_obstacle(self) -> None:
        """
        Display the obstacle
        """
        p1 = self.obstacle.wall_p1
        p2 = self.obstacle.wall_p2
        p3 = self.obstacle.wall_p3
        p4 = self.obstacle.wall_p4
        pygame.draw.line(self.screen, self.BLUE, p1, p2, 1)
        pygame.draw.line(self.screen, self.BLUE, p2, p3, 1)
        pygame.draw.line(self.screen, self.BLUE, p3, p4, 1)
        pygame.draw.line(self.screen, self.BLUE, p4, p1, 1)

        # center = [650,650]
        # radius = 30
        # pygame.draw.circle(self.screen, self.BLUE, center, radius)

    def display_robot(self) -> None:
        """
        Display the robot
        """
        origin = self.robot.world_origin
        j0 = self.robot.world_pos0
        j1 = self.robot.world_pos1

        # Draw the origin
        pygame.draw.circle(self.screen, self.BLACK, origin, 4)
        # Draw link 0
        pygame.draw.line(self.screen, self.BLACK, origin, j0, 2)
        # Draw joint 0
        pygame.draw.circle(self.screen, self.BLACK, j0, 4)
        # Draw link 1
        pygame.draw.line(self.screen, self.BLACK, j0, j1, 2)
        # Draw joint 1
        pygame.draw.circle(self.screen, self.BLACK, j1, 4)

    
    def display_info(self) -> None:
        """
        Display the robot info
        """
        endEffecter_pos = self.robot.all_world_pos1
  
        endEffecter_vel = np.diff(endEffecter_pos,axis = 0) / 0.033

        endEffecter_acc = np.diff(endEffecter_vel,axis = 0) / 0.033


        fig, axs = plt.subplots(1,3,figsize=(12, 3), tight_layout=True)
        axs[0].plot(endEffecter_pos[:,0],endEffecter_pos[:,1])
        axs[0].set_title('Path')
        axs[0].set_xlabel('X position')
        axs[0].set_ylabel('Y position')

        axs[1].plot(endEffecter_vel[:,0], label='X')
        axs[1].plot(endEffecter_vel[:,1], label='Y')
        axs[1].set_title('Velocity vs Time')
        axs[1].set_xlabel('Time')
        axs[1].set_ylabel('Velocity')
        axs[1].legend()

        axs[2].plot(endEffecter_acc[:,0], label='X')
        axs[2].plot(endEffecter_acc[:,1], label='Y')
        axs[2].set_title('Acceleration vs Time')
        axs[2].set_xlabel('Time')
        axs[2].set_ylabel('Acceleration')
        axs[2].legend()

        buf, _ = fig.canvas.print_to_buffer()
        small_screen = pygame.image.fromstring(buf, fig.canvas.get_width_height(), 'RGBA')
        self.screen.blit(small_screen, (0, 0))  
        plt.close()


    def update_display(self, success: bool) -> bool:
        for event in pygame.event.get():
            # Keypress
            if event.type == pygame.KEYDOWN:
                # Escape key
                if event.key == pygame.K_ESCAPE:
                    return False
            # Window Close Button Clicked
            if event.type == pygame.QUIT:
                return False

        self.screen.fill(self.WHITE)
        self.small_screen.fill(self.BLUE)

        self.display_goal_start()

        self.display_obstacle()

        self.display_robot()

        self.display_info()

        if success:
            text = self.font.render('Success!', True, self.BLACK)
            self.screen.blit(text, (1, 1))

        pygame.display.flip()

        return True

    def cleanup(self) -> None:
        pygame.quit()


class Runner:
    def __init__(
        self,
        robot: Robot,
        vis: Visualizer,
        obstacle:Obstacle
    ) -> None:
        self.robot    = robot
        self.vis      = vis
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
            running = self.vis.update_display(success)

            # sleep for Robot DT seconds, to force update rate
            time.sleep(self.robot.DT)


    def check_success(self) -> bool:
        """
        Check that robot's joint 2 is very close to the goal.
        Don't not use exact comparision, to be robust to floating point calculations.
        """
        world_goal = self.robot.world_goal
        endEffector_pos = self.robot.world_pos1

        return np.allclose(endEffector_pos, world_goal, atol=2)

    def cleanup(self) -> None:
        self.vis.cleanup()


def main() -> None:  
    # some parameters
    height = 1200
    width  = 1200
    world_origin  = (int(width / 2), int(height / 2))

    param = {
    "height": 1200, 
    "width":  1200,
    "link_1": 75,
    "link_2": 50,
    "world_origin":world_origin,
    }

    robot = Robot(param)
    robot.generate_random_point()
    obstacle = Obstacle(robot) 
    vis    = Visualizer(param,robot,obstacle)
    runner = Runner(robot, vis,obstacle)

    try:
        runner.run()
    except AssertionError as e:
        print(f'ERROR: {e}, Aborting.')
    except KeyboardInterrupt:
        pass
    finally:
        runner.cleanup()


if __name__ == '__main__':
    main()
