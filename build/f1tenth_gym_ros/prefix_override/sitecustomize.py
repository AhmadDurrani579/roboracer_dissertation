import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/loq/roboracer_ws/roboracer_dissertation/install/f1tenth_gym_ros'
