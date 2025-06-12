import rclpy                                                             # type: ignore
from rclpy.node import Node                                              # type: ignore
from geometry_msgs.msg import Twist                                      # type: ignore
from sensor_msgs.msg import LaserScan           # Für Hinderniserkennung # type: ignore
from rclpy.qos import qos_profile_sensor_data                            # type: ignore
from enum import Enum

import sounddevice as sd    # type: ignore
import queue
import json
import vosk         # type: ignore
import numpy as np  # type: ignore

from nav2_simple_commander.robot_navigator import BasicNavigator    # type: ignore

from geometry_msgs.msg import PoseStamped                           # type: ignore

User = "andy"  # "andy" oder "bastian"
Abstand = 0.3
Timer_callback_Aufrufsintervall = 0.02  
Angle = 20

class Hinderniserkennung(Enum):
    front = 1
    back = 2
    none = 3

class DirectionState(Enum):
    forward = 1
    backward = 2 
    circle = 3
    none = 4

Valid_Commands = {"zurück", "vorwärts", "links", "rechts", "kreis", "halt"}
Valid_point_Commands = {"tür flur", "tür labor", "wand"}

Ausgabe_Befehlsliste = "\nMögliche Befehle: vorwärts, zurück, halt, links, rechts, kreis\n"

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        if User == "andy":
            model_path = r"/home/andy/Turtelbot3_voicecontroll/vosk-model-small-de-0.15"
        elif User == "bastian":
            model_path = r"/home/basti/Schreibtisch/Turtlebot/Voicecontrol/vosk-model-de-0.15"

        self.model = vosk.Model(model_path)
        self.navigator = BasicNavigator()

        self.waypoints_list = {
            "tür flur": (1.25, 3.9),
            "tür labor": (-6.1, -0.95),
            "wand": (-0.9, -1.8)
        }

        self.orientation_list = {
            "tür flur": 1.0,
            "tür labor": 0.0,
            "wand": 0.0
        }

        self.Hindernisserkennung = Hinderniserkennung.none
        self.DirectionState = DirectionState.none
        self.q = queue.Queue()
        self.twist = Twist()
        self.device_id = None

        self.stream = sd.RawInputStream(
            samplerate=16000, blocksize=2048, dtype='int16',
            channels=1, callback=self.audio_callback,
            device=self.device_id)
        self.stream.start()

        self.rec = vosk.KaldiRecognizer(self.model, 16000)

        self.get_logger().info("\n-----Sprachsteuerung gestartet-----\n")
        self.get_logger().info(Ausgabe_Befehlsliste)

        self.timer = self.create_timer(Timer_callback_Aufrufsintervall, self.timer_callback)

        self.obstacle_detected = False
        self.obstacle_handling_active = False

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

# -----------------------------------------------------------------------------------------------
    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Sounddevice Status: {status}")
        self.q.put(bytes(indata))

# -----------------------------------------------------------------------------------------------
    def timer_callback(self):
        while not self.q.empty() and self.Hindernisserkennung == Hinderniserkennung.none:
            data = self.q.get()
            if self.rec.AcceptWaveform(data):
                result = json.loads(self.rec.Result())
                command = result.get("text", "")
                if command in Valid_Commands:
                    self.get_logger().info(f"Gültiger Befehl erkannt: {command}")
                    self.handle_movement_Command(command)
                elif command in Valid_point_Commands:
                    self.get_logger().info(f"Ziel Befehl erkannt: {command}")
                    self.handle_navigation_command(command)

# -----------------------------------------------------------------------------------------------
    def handle_movement_Command(self, text):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        if "vorwärts" in text:
            self.DirectionState = DirectionState.forward
            self.twist.linear.x = 0.5
        elif "zurück" in text:
            self.DirectionState = DirectionState.backward
            self.twist.linear.x = -0.5
        elif "links" in text:
            self.twist.angular.z = 0.2
        elif "rechts" in text:
            self.twist.angular.z = -0.2
        elif "kreis" in text:
            self.DirectionState = DirectionState.circle
            self.twist.linear.x = 0.3
            self.twist.angular.z = -0.6
        elif "halt" in text:
            self.get_logger().info("\n-----Warte auf neuen Sprachbefehl-----\n")
            self.get_logger().info(Ausgabe_Befehlsliste)
        else:
            return
        self.pub.publish(self.twist)

# -----------------------------------------------------------------------------------------------
    def handle_navigation_command(self, ziel_name):
        coords = self.waypoints_list.get(ziel_name)
        orientation = self.orientation_list.get(ziel_name, 1.0)

        if coords:
            x, y = coords
            self.navigate_to_pose(x, y, orientation)

# -----------------------------------------------------------------------------------------------
    def scan_callback(self, msg):
        num_ranges = len(msg.ranges)
        front_center = num_ranges
        window_front = msg.ranges[max(0, front_center - Angle):min(num_ranges, front_center + Angle)]
        back_center = num_ranges // 2
        window_back = msg.ranges[max(0, back_center - Angle):min(num_ranges, back_center + Angle)]

        valid_ranges_front = [r for r in window_front if np.isfinite(r) and r > 0.05]
        valid_ranges_back = [r for r in window_back if np.isfinite(r) and r > 0.05]

        if not valid_ranges_front or not valid_ranges_back:
            return

        min_distance_front = min(valid_ranges_front)
        min_distance_back = min(valid_ranges_back)

        if min_distance_back <= Abstand:
            self.Hindernisserkennung = Hinderniserkennung.back
        elif min_distance_front <= Abstand:
            self.Hindernisserkennung = Hinderniserkennung.front
        else:
            if self.Hindernisserkennung != Hinderniserkennung.none:
                self.get_logger().info("\n\nKein Hindernis mehr im Weg\n")
                self.get_logger().info(Ausgabe_Befehlsliste)
                self.Hindernisserkennung = Hinderniserkennung.none
                self.DirectionState = DirectionState.none
                stopTwist = Twist()
                self.pub.publish(stopTwist)
                self.obstacle_handling_active = False

        if self.Hindernisserkennung == Hinderniserkennung.front and self.DirectionState in [DirectionState.forward, DirectionState.circle]:
            stopTwist = Twist()
            self.pub.publish(stopTwist)
            self.twist.linear.x = -0.2
            self.pub.publish(self.twist)

            if not self.obstacle_handling_active:
                self.get_logger().warn(f"\n\n!!!!Hindernis Vorne erkannt in {min_distance_front:.2f} m  Hält an!\n")
                self.get_logger().info("\n\nRückwärts fahren bis kein Hindernis mehr im Weg\n")
                self.obstacle_handling_active = True

        elif self.Hindernisserkennung == Hinderniserkennung.back and self.DirectionState in [DirectionState.backward, DirectionState.circle]:
            stopTwist = Twist()
            self.pub.publish(stopTwist)
            self.twist.linear.x = 0.2
            self.pub.publish(self.twist)

            if not self.obstacle_handling_active:
                self.get_logger().warn(f"\n\n!!!!Hindernis Hinten erkannt in {min_distance_back:.2f} m  Hält an!\n")
                self.get_logger().info("\n\nVorwärts fahren bis kein Hindernis mehr im Weg\n")
                self.obstacle_handling_active = True

# -----------------------------------------------------------------------------------------------
    def navigate_to_pose(self, x, y, orientation_w):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = orientation_w

        self.get_logger().info(f"Navigiere zu: x={x}, y={y}, w={orientation_w}")
        self.navigator.goToPose(goal_pose)

# -----------------------------------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node gestoppt.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
