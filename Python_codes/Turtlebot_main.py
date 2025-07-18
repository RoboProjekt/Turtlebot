import rclpy                                #type: ignore
import sys

# absolute_src_path = '/home/pi/turtlebot_ws/src'    #gilt nur auf PI
# sys.path.append(absolute_src_path)

from voice_control import VoiceControlNode   #type: ignore
from battery_warning import BatteryMonitor   #type: ignore
# from darknet_publisher.darknet_pub import YoloPublisher



def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceControlNode()
    battery_warning = BatteryMonitor()
    # object_detect_node = YoloPublisher()
    try:
        rclpy.spin(voice_node)
        rclpy.spin(battery_warning)
        # rclpy.spin(object_detect_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_node.destroy_node()
        battery_warning.destroy_node()
        # object_detect_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
