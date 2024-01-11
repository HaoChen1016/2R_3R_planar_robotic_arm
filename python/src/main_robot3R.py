"""
challenge.py
"""
from typing import List, Tuple, Union
from robot import *
from visualizer import *
from runner import *

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
    "link_3": 25,
    "world_origin":world_origin,
    }

    robot = Robot(param)
    # robot.generate_random_goal()
    # print(robot.world_goal)
    obstacle = Obstacle(robot) 
    vis      = Visualizer(param,robot,obstacle)
    runner   = Runner(robot, vis,obstacle)

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
