import rclpy                                # type:ignore
from rclpy.node import Node                 # type:ignore
from std_msgs.msg import Float32            # type:ignore
from sensor_msgs.msg import BatteryState    # type:ignore
from turtlebot3_msgs.srv import Sound       # type:ignore
import time

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')

        # Schwelle für Warnung
        self.warning_threshold = 0.5  # 50 %
        self.warning_interval = 120   # alle 120 s
        self.last_warning_time = 0.0

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


        # Timer für regelmäßige Statusausgabe (alle 60s)
        self.status_timer = self.create_timer(
            60,
            self.status_output_callback
        )

    def play_sound(self, sound_value=2):
        request = Sound.Request()
        request.value = sound_value
        self.sound_client.call_async(request)

    def battery_callback(self, msg: BatteryState):
        percentage = msg.percentage
        self.last_percentage = percentage
        now = time.time()

        if percentage < self.warning_threshold and now - self.last_warning_time > self.warning_interval:
            self.get_logger().warn('🔋 Akku fast leer! Warnung wird ausgegeben.')
            self.play_sound(3)
            self.last_warning_time = now

    def status_output_callback(self):
        if self.last_percentage is not None:
            self.get_logger().info(f'⏱ Regelmäßige Ausgabe: Akkuladung bei {self.last_percentage:.1f}%')
        else:
            self.get_logger().info('⏱ Noch kein Akkustand verfügbar.')

