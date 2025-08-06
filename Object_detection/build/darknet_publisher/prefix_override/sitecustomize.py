import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/basti/Schreibtisch/Turtlebot/Object_detection/install/darknet_publisher'
