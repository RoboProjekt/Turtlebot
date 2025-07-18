import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import BatteryState
from turtlebot3_msgs.srv import Sound
import time


class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # Parameter
        self.warning_threshold = 0.2  # 20 %
        self.warning_interval = 120   # alle 120 s
        self.last_warning_time = 0.0

        # Sound-Client
        self.sound_client = self.create_client(Sound, '/sound')
        
        # Subscriber auf Akkustand
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/battery_state',
            self.battery_callback,
            10
        )

    def play_sound(self, sound_value=2):
        request = Sound.Request()
        request.value = sound_value
        future = self.sound_client.call_async(request)


    def battery_callback(self, msg: BatteryState):
        percentage = msg.percentage
        now = time.time()

        self.get_logger().info(f'Akkuladung: {percentage * 100:.1f}%')

        if percentage < self.warning_threshold and now - self.last_warning_time > self.warning_interval:
            self.get_logger().warn('🔋 Akku fast leer! Warnung wird ausgegeben.')
            self.play_sound(3)
            self.last_warning_time = now

