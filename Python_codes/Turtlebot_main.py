import rclpy                                        # type: ignore
from rclpy.executors import MultiThreadedExecutor   #type:ignore
import sys

from voice_control_basti_040825 import VoiceControlNodeBasti #type:ignore
from battery_warning import BatteryMonitor   # type: ignore


def main(args=None):
    rclpy.init(args=args)

    # Nodes initialisieren
    voice_node_basti = VoiceControlNodeBasti()
    battery_warning = BatteryMonitor()

    # Multi-Threaded-Executor für parallele Verarbeitung
    executor = MultiThreadedExecutor()
    executor.add_node(battery_warning)
    executor.add_node(voice_node_basti)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        voice_node_basti.destroy_node()
        battery_warning.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
