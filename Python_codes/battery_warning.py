import rclpy                                # type:ignore
from rclpy.node import Node                 # type:ignore
from std_msgs.msg import Float32            # type:ignore
from sensor_msgs.msg import BatteryState    # type:ignore
from turtlebot3_msgs.srv import Sound       # type:ignore
import time

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # Letzter bekannter Akkustand
        self.last_percentage = None

        # Sound-Client
        self.sound_client = self.create_client(Sound, '/sound')

        # Subscriber auf Akkustand
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

        self.get_logger().info('🔋 BatteryMonitor-Node gestartet.')

        # Timer für regelmäßige Statusausgabe (alle 180s)
        self.status_timer = self.create_timer(
            180,
            self.status_output_callback
        )

    def __del__(self):
            self.get_logger().info("BateryMonitor wird zerstört!")

    def play_sound(self, sound_value=2):
        request = Sound.Request()
        request.value = sound_value
        future = self.sound_client.call_async(request)


    def status_output_callback(self):
        if self.last_percentage is not None:
            self.get_logger().info(f'Momentane Akkuladung bei 🔋 {self.last_percentage:.1f}%')
        else:
            self.get_logger().info('Noch kein Akkustand verfügbar.')

