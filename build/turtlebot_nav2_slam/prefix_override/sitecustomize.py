import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/anurag-mittal/nav2_slam_ws/install/turtlebot_nav2_slam'
