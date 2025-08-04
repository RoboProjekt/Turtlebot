import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import re

class YoloPublisher(Node):
    def __init__(self):
        super().__init__('yolo_object_publisher')
        self.publisher_ = self.create_publisher(String, 'yolo_objects', 10)

        # Starte Darknet als Subprozess
        self.process = subprocess.Popen(
            ['home/pi/darknet/darknet', 'detector', 'demo', 'data/yolo-aiv2/obj.data',
             'data/yolo-aiv2/yolov4-tiny-custom.cfg', 'data/yolo-aiv2/yolov4-tiny-custom_best.weights',
             '-c', '0', '-thresh', '0.7'],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )

        self.get_logger().info('YOLO Detection gestartet...')
        self.read_darknet_output()

        #def __del__(self):
         #   self.get_logger().info("YoloPublisher wird zerstört!")

    def read_darknet_output(self):
        pattern = re.compile(r"(\w+): (\d+)%")  # z.B. "door: 78%"

        for line in self.process.stdout:
            match = pattern.findall(line)
            if match:
                for obj, conf in match:
                    msg = f"{obj} {conf}%"
                    ros_msg = String()
                    ros_msg.data = msg
                    self.publisher_.publish(ros_msg)
                    self.get_logger().info(f'Gesendet: {msg}')


