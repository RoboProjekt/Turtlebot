import rclpy                                        # type: ignore
from rclpy.executors import MultiThreadedExecutor   #type:ignore
import sys

from voice_control_basti_040825 import VoiceControlNodeBasti #New
from voice_control import VoiceControlNode   # type: ignore
from battery_warning import BatteryMonitor   # type: ignore
# from darknet_publisher.darknet_pub import YoloPublisher

def main(args=None):
    rclpy.init(args=args)

    # Nodes initialisieren
    #voice_node = VoiceControlNode()
    voice_node_basti = VoiceControlNodeBasti()
    battery_warning = BatteryMonitor()
    # object_detect_node = YoloPublisher()

    # Multi-Threaded-Executor für parallele Verarbeitung
    executor = MultiThreadedExecutor()
    executor.add_node(battery_warning)
    #executor.add_node(voice_node)
    executor.add_node(voice_node_basti)
    # executor.add_node(object_detect_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        #voice_node.destroy_node()
        voice_node_basti.destroy_node()
        battery_warning.destroy_node()
        # object_detect_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
