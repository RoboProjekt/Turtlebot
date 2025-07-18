
import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from geometry_msgs.msg import Twist, PoseStamped, Quaternion  # type: ignore
from sensor_msgs.msg import LaserScan  # type: ignore
from rclpy.qos import qos_profile_sensor_data  # type: ignore
from enum import Enum

import sounddevice as sd  # type: ignore
import queue
import json
import vosk  # type: ignore
import numpy as np  # type: ignore
import math

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult  # type: ignore
from std_msgs.msg import String  # type: ignore

# User configuration
# Welcher Nutzer-Pfad für das Vosk-Modell verwendet wird: 'andy', 'bastian' oder 'pi'
default_user = "andy"  # wählt den Vosk-Modell-Pfad basierend auf Nutzer
# Abtastrate (Hz) für Audioaufnahme (Standard: 44100 für Raspberry Pi)
samplerate_number = 44100
# Anzahl der Frames pro Audioblock (Empfehlung: 4096)
blocksize_number = 4096
# Sicherheitsabstand (m): Abstand, der vor erkannten Objekten eingehalten wird
obstacle_distance = 0.3
# Timer-Intervall (s) für Spracherkennungs-Callback (Beeinflusst Reaktionszeit)
timer_interval = 0.02
# Halbwinkel (Grad) um Front-Richtung für LIDAR-Abstandsmessung
scan_angle = 20

class Hinderniserkennung(Enum):
    front = 1
    back = 2
    none = 3

class DirectionState(Enum):
    forward = 1
    backward = 2
    circle = 3
    none = 4

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.pub_cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_yolo = self.create_subscription(String, 'yolo_objects', self.yolo_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        
        # Vereinte Waypoints: vordefiniert + erkannte Türen
        # Format: name -> (x, y, yaw)
        self.waypoints = {
            "tür flur": (1.25, 3.9, 1.57),
            "tür labor": (-6.1, -0.95, 0.0),
            "wand": (-0.9, -1.8, 0.0)
        }

        self.latest_scan = None

        # Sprach-Setup
        model_path = self._get_model_path(default_user)
        self.model = vosk.Model(model_path)
        self.rec = vosk.KaldiRecognizer(self.model, 16000)
        self.q_audio = queue.Queue()
        self.stream = sd.RawInputStream(samplerate=samplerate_number, blocksize=blocksize_number,
                                        dtype='int16', channels=1, callback=self.audio_callback)
        self.stream.start()

        # Navigation
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.obstacle_state = Hinderniserkennung.none
        self.move_state = DirectionState.none
        self.obstacle_handling = False

        self.create_timer(timer_interval, self.timer_callback)
        self.get_logger().info("Sprachsteuerung mit Tür-Erkennung gestartet.")

    def _get_model_path(self, user):
        if user == "andy":
            return "/home/andy/.../vosk-model-small-de-0.15"
        elif user == "bastian":
            return "/home/basti/.../vosk-model-small-de-0.15"
        else:
            return "/home/pi/.../vosk-model-small-de-0.15"

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Audio-Status: {status}")
        self.q_audio.put(bytes(indata))

    def timer_callback(self):
        while not self.q_audio.empty():
            data = self.q_audio.get()
            if self.rec.AcceptWaveform(data):
                result = json.loads(self.rec.Result())
                cmd = result.get('text','').lower()
                if cmd in {"vorwärts","zurück","links","rechts","kreis","halt"}:
                    self.handle_movement(cmd)
                elif cmd.startswith("tür "):
                    self.handle_navigation(cmd)

    def handle_movement(self, cmd):
        twist = Twist()
        if cmd == "vorwärts":
            twist.linear.x = 0.5; self.move_state = DirectionState.forward
        elif cmd == "zurück":
            twist.linear.x = -0.5; self.move_state = DirectionState.backward
        elif cmd == "links":
            twist.angular.z = 0.2
        elif cmd == "rechts":
            twist.angular.z = -0.2
        elif cmd == "kreis":
            twist.linear.x = 0.3; twist.angular.z = -0.6
        self.pub_cmd.publish(twist)

    def handle_navigation(self, target_cmd):
        # Nutzt vereinte self.waypoints
        if target_cmd in self.waypoints:
            x,y,yaw = self.waypoints[target_cmd]
            self.navigate_to(x,y,yaw)
        else:
            self.get_logger().warn(f"Unbekanntes Ziel: {target_cmd}")

    def scan_callback(self, scan):
        self.latest_scan = scan
        # Hinderniserkennung kann hier weiter genutzt werden

    def yolo_callback(self, msg: String):
        text = msg.data.lower()
        if "door" in text or "tür" in text:
            if not self.latest_scan:
                return
            # Abstand inkl. Sicherheit abziehen
            raw_dist = self._get_front_distance(self.latest_scan)
            dist = max(0.0, raw_dist - obstacle_distance)  # Sicherheitsabstand abziehen
            if not np.isfinite(dist):
                return
            x,y = self._calc_global_position(dist)
            # Prüfen, ob Position schon in waypoints existiert
            for (vx,vy,yaw) in self.waypoints.values():
                if math.hypot(vx-x, vy-y) < obstacle_distance:
                    return  # bereits bekannt
            # Neue Tür anlegen
            name = f"tür {len(self.waypoints)+1}"
            yaw = self._get_current_yaw()
            self.waypoints[name] = (x,y,yaw)
            self.get_logger().info(f"Neue Tür erkannt: {name} at ({x:.2f},{y:.2f})")

    def _get_front_distance(self, scan: LaserScan) -> float:
        mid = len(scan.ranges)//2
        window = scan.ranges[mid-scan_angle:mid+scan_angle]
        valid = [r for r in window if np.isfinite(r) and r>0]
        return min(valid) if valid else float('nan')

    def _calc_global_position(self, dist: float):
        pose = self.navigator.getCurrentPose()
        q = pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        x0,y0 = pose.pose.position.x, pose.pose.position.y
        return x0 + dist*math.cos(yaw), y0 + dist*math.sin(yaw)

    def _get_current_yaw(self):
        pose = self.navigator.getCurrentPose()
        q = pose.pose.orientation
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))

    def navigate_to(self, x: float, y: float, yaw: float):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation = self._euler_to_quat(0,0,yaw)
        self.navigator.goToPose(goal)
        start = self.get_clock().now().nanoseconds
        while not self.navigator.isTaskComplete():
            if (self.get_clock().now().nanoseconds - start)/1e9 > 300:
                self.navigator.cancelTask()
                break
        result = self.navigator.getResult()
        self.get_logger().info("Navigation result: %s" % result)

    def _euler_to_quat(self, roll,pitch,yaw) -> Quaternion:
        qx = math.sin(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) - math.cos(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        qy = math.cos(roll/2)*math.sin(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.cos(pitch/2)*math.sin(yaw/2)
        qz = math.cos(roll/2)*math.cos(pitch/2)*math.sin(yaw/2) - math.sin(roll/2)*math.sin(pitch/2)*math.cos(yaw/2)
        qw = math.cos(roll/2)*math.cos(pitch/2)*math.cos(yaw/2) + math.sin(roll/2)*math.sin(pitch/2)*math.sin(yaw/2)
        return Quaternion(x=qx,y=qy,z=qz,w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node beendet.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    
    

