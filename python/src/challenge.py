"""
challenge.py
"""
import time
from typing import List, Tuple, Union

import numpy as np
import pygame


class Robot:
    JOINT_LIMITS = [-6.28, 6.28]
    MAX_VELOCITY = 15
    MAX_ACCELERATION = 50
    DT = 0.033

    link_1: float = 75.  # pixels
    link_2: float = 50.  # pixels
    _theta_0: float      # radians
    _theta_1: float      # radians

    def __init__(self) -> None:
        # internal variables
        self.all_theta_0: List[float] = []
        self.all_theta_1: List[float] = []

        self.theta_0 = 0.
        self.theta_1 = 0.

    # Getters/Setters
    @property
    def theta_0(self) -> float:
        return self._theta_0

    @theta_0.setter
    def theta_0(self, value: float) -> None:
        self.all_theta_0.append(value)
        self._theta_0 = value
        # Check limits
        assert self.check_angle_limits(value), \
            f'Joint 0 value {value} exceeds joint limits'
        assert self.max_velocity(self.all_theta_0) < self.MAX_VELOCITY, \
            f'Joint 0 Velocity {self.max_velocity(self.all_theta_0)} exceeds velocity limit'
        assert self.max_acceleration(self.all_theta_0) < self.MAX_ACCELERATION, \
            f'Joint 0 Accel {self.max_acceleration(self.all_theta_0)} exceeds acceleration limit'

    @property
    def theta_1(self) -> float:
        return self._theta_1

    @theta_1.setter
    def theta_1(self, value: float) -> None:
        self.all_theta_1.append(value)
        self._theta_1 = value
        assert self.check_angle_limits(value), \
            f'Joint 1 value {value} exceeds joint limits'
        assert self.max_velocity(self.all_theta_1) < self.MAX_VELOCITY, \
            f'Joint 1 Velocity {self.max_velocity(self.all_theta_1)} exceeds velocity limit'
        assert self.max_acceleration(self.all_theta_1) < self.MAX_ACCELERATION, \
            f'Joint 1 Accel {self.max_acceleration(self.all_theta_1)} exceeds acceleration limit'

    # Kinematics
    def joint_1_pos(self) -> Tuple[float, float]:
        """
        Compute the x, y position of joint 1
        """
        return self.link_1 * np.cos(self.theta_0), self.link_1 * np.sin(self.theta_0)

    def joint_2_pos(self) -> Tuple[float, float]:
        """
        Compute the x, y position of joint 2
        """
        return self.forward(self.theta_0, self.theta_1)

    @classmethod
    def forward(cls, theta_0: float, theta_1: float) -> Tuple[float, float]:
        """
        Compute the x, y position of the end of the links from the joint angles
        """
        x = cls.link_1 * np.cos(theta_0) + cls.link_2 * np.cos(theta_0 + theta_1)
        y = cls.link_1 * np.sin(theta_0) + cls.link_2 * np.sin(theta_0 + theta_1)

        return x, y

    @classmethod
    def inverse(cls, x: float, y: float) -> Tuple[float, float]:
        """
        Compute the joint angles from the position of the end of the links
        """
        theta_1 = np.arccos((x ** 2 + y ** 2 - cls.link_1 ** 2 - cls.link_2 ** 2)
                            / (2 * cls.link_1 * cls.link_2))
        theta_0 = np.arctan2(y, x) - \
            np.arctan((cls.link_2 * np.sin(theta_1)) /
                      (cls.link_1 + cls.link_2 * np.cos(theta_1)))

        return theta_0, theta_1

    @classmethod
    def check_angle_limits(cls, theta: float) -> bool:
        return cls.JOINT_LIMITS[0] < theta < cls.JOINT_LIMITS[1]

    @classmethod
    def max_velocity(cls, all_theta: List[float]) -> float:
        return float(max(abs(np.diff(all_theta) / cls.DT), default=0.))

    @classmethod
    def max_acceleration(cls, all_theta: List[float]) -> float:
        return float(max(abs(np.diff(np.diff(all_theta)) / cls.DT / cls.DT), default=0.))

    @classmethod
    def min_reachable_radius(cls) -> float:
        return max(cls.link_1 - cls.link_2, 0)

    @classmethod
    def max_reachable_radius(cls) -> float:
        return cls.link_1 + cls.link_2


class World:
    def __init__(
        self,
        width: int,
        height: int,
        robot_origin: Tuple[int, int],
        goal: Tuple[int, int]
    ) -> None:
        self.width = width
        self.height = height
        self.robot_origin = robot_origin
        self.goal = goal

    def convert_to_display(
            self, point: Tuple[Union[int, float], Union[int, float]]) -> Tuple[int, int]:
        """
        Convert a point from the robot coordinate system to the display coordinate system
        """
        robot_x, robot_y = point
        offset_x, offset_y = self.robot_origin

        return int(offset_x + robot_x), int(offset_y - robot_y)


class Visualizer:
    BLACK: Tuple[int, int, int] = (0, 0, 0)
    RED: Tuple[int, int, int] = (255, 0, 0)
    WHITE: Tuple[int, int, int] = (255, 255, 255)

    def __init__(self, world: World) -> None:
        """
        Note: while the Robot and World have the origin in the center of the
        visualization, rendering places (0, 0) in the top left corner.
        """
        pygame.init()
        pygame.font.init()
        self.world = world
        self.screen = pygame.display.set_mode((world.width, world.height))
        pygame.display.set_caption('Gherkin Challenge')
        self.font = pygame.font.SysFont('freesansbolf.tff', 30)

    def display_world(self) -> None:
        """
        Display the world
        """
        goal = self.world.convert_to_display(self.world.goal)
        pygame.draw.circle(self.screen, self.RED, goal, 6)

    def display_robot(self, robot: Robot) -> None:
        """
        Display the robot
        """
        j0 = self.world.robot_origin
        j1 = self.world.convert_to_display(robot.joint_1_pos())
        j2 = self.world.convert_to_display(robot.joint_2_pos())
        # Draw joint 0
        pygame.draw.circle(self.screen, self.BLACK, j0, 4)
        # Draw link 1
        pygame.draw.line(self.screen, self.BLACK, j0, j1, 2)
        # Draw joint 1
        pygame.draw.circle(self.screen, self.BLACK, j1, 4)
        # Draw link 2
        pygame.draw.line(self.screen, self.BLACK, j1, j2, 2)
        # Draw joint 2
        pygame.draw.circle(self.screen, self.BLACK, j2, 4)

    def update_display(self, robot: Robot, success: bool) -> bool:
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

        self.display_world()

        self.display_robot(robot)

        if success:
            text = self.font.render('Success!', True, self.BLACK)
            self.screen.blit(text, (1, 1))

        pygame.display.flip()

        return True

    def cleanup(self) -> None:
        pygame.quit()


class Controller:
    def __init__(self, goal: Tuple[int, int]) -> None:
        self.goal = goal
        self.goal_theta_0, self.goal_theta_1 = Robot.inverse(self.goal[0], self.goal[1])

    def step(self, robot: Robot) -> Robot:
        """
        Simple P controller
        """
        theta_0_error = self.goal_theta_0 - robot.theta_0
        theta_1_error = self.goal_theta_1 - robot.theta_1

        robot.theta_0 += theta_0_error / 10
        robot.theta_1 += theta_1_error / 10

        return robot


class Runner:
    def __init__(
        self,
        robot: Robot,
        controller: Controller,
        world: World,
        vis: Visualizer
    ) -> None:
        self.robot = robot
        self.controller = controller
        self.world = world
        self.vis = vis

    def run(self) -> None:
        running = True

        while running:
            # Step the controller
            self.robot = self.controller.step(self.robot)

            # Check collisions
            # TODO

            # Check success
            success = self.check_success(self.robot, self.world.goal)

            # Update the display
            running = self.vis.update_display(self.robot, success)

            # sleep for Robot DT seconds, to force update rate
            time.sleep(self.robot.DT)

    @staticmethod
    def check_success(robot: Robot, goal: Tuple[int, int]) -> bool:
        """
        Check that robot's joint 2 is very close to the goal.
        Don't not use exact comparision, to be robust to floating point calculations.
        """
        return np.allclose(robot.joint_2_pos(), goal, atol=0.25)

    def cleanup(self) -> None:
        self.vis.cleanup()


def generate_random_goal(min_radius: float, max_radius: float) -> Tuple[int, int]:
    """
    Generate a random goal that is reachable by the robot arm
    """
    # Ensure theta is not 0
    theta = (np.random.random() + np.finfo(float).eps) * 2 * np.pi
    # Ensure point is reachable
    r = np.random.uniform(low=min_radius, high=max_radius)

    x = int(r * np.cos(theta))
    y = int(r * np.sin(theta))

    return x, y


def main() -> None:
    height = 300
    width = 300

    robot_origin = (int(width / 2), int(height / 2))
    goal = generate_random_goal(Robot.min_reachable_radius(), Robot.max_reachable_radius())

    robot = Robot()
    controller = Controller(goal)
    world = World(width, height, robot_origin, goal)
    vis = Visualizer(world)

    runner = Runner(robot, controller, world, vis)

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
