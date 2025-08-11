import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/basti/turtlebot3_ws/src/darknet_publisher/install/darknet_publisher'
