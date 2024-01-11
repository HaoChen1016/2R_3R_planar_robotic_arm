
from typing import List, Tuple, Union
from robot import *
from obstacle import *
import time
import numpy as np
import pygame
import matplotlib.pyplot as plt


class  Visualizer:
    BLACK: Tuple[int, int, int] = (0, 0, 0)
    RED:   Tuple[int, int, int] = (255, 0, 0)
    WHITE: Tuple[int, int, int] = (255, 255, 255)
    BLUE:  Tuple[int, int, int] = (0, 0, 205)

    def __init__(self,param:dict, robot:Robot,obstacle:Obstacle) -> None:
        """
        Note: while the Robot and World have the origin in the center of the
        visualization, rendering places (0, 0) in the top left corner.
        """
        height = param["height"]
        width  = param["width"]
        self.robot    = robot
        self.obstacle = obstacle

        pygame.init()
        pygame.font.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption('Gherkin Challenge')
        self.font = pygame.font.SysFont('freesansbolf.tff', 30)

        small_window_size = (200, 100)
        self.small_screen = pygame.Surface(small_window_size)
        

    def display_goal(self) -> None:
        """
        Display the goal position
        """
        pygame.draw.circle(self.screen, self.RED, self.robot.world_goal[0:2], 6)
    
    def display_obstacle(self) -> None:
        """
        Display the obstacle
        """
        # p1 = [300,300]
        # p2 = [300,200]
        # pygame.draw.line(self.screen, self.BLUE, p1, p2, 10)

        # center = [650,650]
        # radius = 30
        # pygame.draw.circle(self.screen, self.BLUE, center, radius)
        
        p1 = self.obstacle.wall_p1
        p2 = self.obstacle.wall_p2
        p3 = self.obstacle.wall_p3
        p4 = self.obstacle.wall_p4
        pygame.draw.line(self.screen, self.BLUE, p1, p2, 1)
        pygame.draw.line(self.screen, self.BLUE, p2, p3, 1)
        pygame.draw.line(self.screen, self.BLUE, p3, p4, 1)
        pygame.draw.line(self.screen, self.BLUE, p4, p1, 1)


    def display_robot(self) -> None:
        """
        Display the robot
        """
        origin = self.robot.world_origin
        j0 = self.robot.world_pos0
        j1 = self.robot.world_pos1
        j2 = self.robot.world_pos2

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
        # Draw link 2
        pygame.draw.line(self.screen, self.BLACK, j1, j2, 2)
        # Draw joint 2
        pygame.draw.circle(self.screen, self.BLACK, j2, 4)

    
    def display_info(self) -> None:
        """
        Display the robot info
        """
        endEffecter_pos = self.robot.all_world_pos2
  
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
        self.small_screen.fill(self.BLUE)

        self.display_goal()

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