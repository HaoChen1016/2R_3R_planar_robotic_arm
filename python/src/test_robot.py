import unittest
import numpy as np
from robot import Robot


class TestEmployee(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        print('setupClass')

    @classmethod
    def tearDownClass(cls):
        print('teardownClass')

    def setUp(self):
        print('setUp')
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

        self.robot = Robot(param)

    def tearDown(self):
        print('tearDown\n')

    def test_check_angle_limits(self):
        print('test_joint1Pos')
        self.assertEqual(self.robot.check_angle_limits(np.pi), True)
    
    '''
    more tests needs to be done!
    '''

if __name__ == '__main__':
    unittest.main()