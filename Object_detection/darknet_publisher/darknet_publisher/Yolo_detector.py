import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data  # type: ignore

from geometry_msgs.msg import Twist  # type: ignore
from sensor_msgs.msg import LaserScan  # Für Hinderniserkennung # type: ignore
from turtlebot3_msgs.srv import Sound #type: ignore
from std_msgs.msg import String  # type: ignore
from geometry_msgs.msg import PoseStamped  # type: ignore
from geometry_msgs.msg import Quaternion  # type: ignore

from enum import Enum
from std_srvs.srv import SetBool                #type:ignore
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult  # type: ignore
from tf2_ros import Buffer, TransformListener #, LookupException, ConnectivityException, ExtrapolationException  ->Nur nötig bei Fehlerausgabe

import sounddevice as sd  # type: ignore
import queue
import json
import vosk  # type: ignore
import numpy as np  # type: ignore
import math
import time
import threading
import subprocess
import re


# Variablen für Funktionen
Abstand = 0.5                               # Abstand in Metern, bei dem ein Hindernis erkannt wird
Timer_callback_Aufrufsintervall = 0.02
Angle = 10                                  # gescannter Winkel in Grad
Angle_doorscan = 2                          # gescannter Winkel für Türerkennung
tracking_radius = 1                         #radius für start/endposition wand tracking


class Hinderniserkennung(Enum):
    front = 1
    back = 2
    none = 3

class DirectionState(Enum):
    forward = 1
    backward = 2
    circle = 3
    none = 4


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Objekterkennung gestartet...')
        #INCLUDE YOLO OBJECT DETECTION
        self.sub_yolo = self.create_subscription(String, 'yolo_objects', self.yolo_callback, 10)

        #Neuer Subscriber für map und baselink zum Umrechnen in current_position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(Timer_callback_Aufrufsintervall, self.timer_callback)
        self.create_timer(Timer_callback_Aufrufsintervall, self.tracking_loop)

        self.waypoints = {
           # "flur": (1.25, 3.9),
             "eingang vorne": (18.5, 1.72, 4.71),
             "eingang mitte": (6.05, 1.77, 4.71),
             "eingang hinten": (-1.7, 1.57, 4.71),
             "stellplatz": (4.2, 1.07, 3.14),
             "start": (0.0, 0.0, 0.00),
             "flur anfang": (18.55, 3.62, 3.14),
             "flur mitte": (6.95, 3.37, 3.14),
             "flur ende": (-9.05, 3.62, 0.00),
             "eingang büro": (18.7, -3.28, 3.14),
             "sofa": (6.6, -0.23, 0.0),
        }

        #NEW SCAN FOR DOOR LOCATION
        self.latest_scan = None
        self.min_distance_front = float('inf')
        self.min_distance_back = float('inf')
        self.min_distance_left = float('inf')
        self.min_distance_right = float('inf')
        self.current_pose = None

        self.navigator = BasicNavigator()

        self.navigator.waitUntilNav2Active()

        self.tracking_active = False
        self.navigating = False
        self.working = False    #Wird gerade ein Befehl ausgeführt? Wenn ja, dann kein neues Kommando, muss erst bei Abschließen der Funktion wieder Freigegeben werden 
                                #(tracking zurück bei startpunkt, navigation bei position angekommen oder abgebrochen)

        self.sound_client = self.create_client(Sound, '/sound')

        self.q = queue.Queue()
        self.twist = Twist()

        # Hinderniserkennung
        self.obstacle_detected = False                  # Zur erkennung ob Hindernis erkannt wurde
        self.obstacle_handling_active = False           # Hinderniserkennungs Handling Status

        self.scan_sub = self.create_subscription(       # LIDAR Scan
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )


    def __del__(self):
        self.get_logger().info("YoloDetector wird zerstört!")

    def timer_callback(self):
        self.current_pose = self.get_robot_pose_map()


    def play_sound(self, sound_value=2):
        request = Sound.Request()
        request.value = sound_value
        future = self.sound_client.call_async(request)


    def output_commands(self):
        self.get_logger().info("\nGültige Befehle: \nWand tracking\n")

        for i, name in enumerate(self.waypoints.keys(), start=1):
            self.get_logger().info(f"{i}: {name}")


    #Kommandozeilen Verarbeitung
    def detect_command(self):
        if not self.working:
            self.output_commands()
            eingabe = input("\nBefehl: ").strip().lower()
            if eingabe == "wand tracking":
                self.get_logger().info(f"Gültiger Befehl erkannt: {eingabe}")
                self.tracking_active = True
                self.working = True
            elif eingabe in self.waypoints:
                self.get_logger().info(f"Gültiger Befehl erkannt: {eingabe}")
                self.navigating = True
                self.working = True
                self.handle_navigation_command(eingabe)
                self.working = False
            else:
                self.get_logger().warn(f"Ungültiger Befehl: {eingabe}")
                self.working = False


    def command_loop(self):
        while rclpy.ok():
            if not self.working:
                self.detect_command()   # blockiert hier, aber ROS spin läuft weiter
            time.sleep(0.1)

    #WAND VERFOLGUNG
    def tracking(self):

        if self.current_pose is None:
            return

        Abstand = 0.5  # Abstand zum Objekt in Metern
        Rückkehr_Radius = 1.0  # Radius zur Erkennung der Startposition

        # --- 1. Startposition einmalig speichern ---
        if not hasattr(self, 'start_pose') and self.min_distance_front <= Abstand:
            self.start_pose = self.current_pose
            self.get_logger().info(f"Starte Tracking. Startposition: x={self.start_pose.x:.2f}, y={self.start_pose.y:.2f}")
            return  # Warte bis zum nächsten Zyklus, um Bewegung zu starten

        # --- 2. Prüfen, ob Startposition wieder erreicht wurde ---
        dx = self.current_pose.x - self.start_pose.x
        dy = self.current_pose.y - self.start_pose.y
        distance_to_start = (dx ** 2 + dy ** 2) ** 0.5

        if distance_to_start <= Rückkehr_Radius:
            self.get_logger().info("Zurück am Startpunkt. Tracking beendet.")
            # Optional: Stoppen
            self.twist.linear.x = 0.0
            self.twist.angular.z = 0.0
            self.pub.publish(self.twist)
            # Optional: Zustandsflag setzen, damit kein weiteres Tracking passiert
            self.tracking_active = False
            self.working = False
            self.play_sound(1)
            return

        # --- 3. Bewegung basierend auf Abstandssensoren ---
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        if self.min_distance_front > Abstand:
            self.twist.linear.x = 0.5

        elif self.min_distance_front <= Abstand:
            if self.min_distance_left > Abstand:
                self.twist.angular.z = 0.5
            else:
                self.twist.linear.x = 0.2
                if self.min_distance_left < Abstand:
                    self.twist.angular.z = 0.2
                elif self.min_distance_left > Abstand:
                    self.twist.angular.z = -0.2
                else:
                    self.twist.angular.z = 0.0

        self.pub.publish(self.twist)


    #Funktion zum wiederholten Aufruf von tracking
    def tracking_loop(self):
        if self.working and self.tracking_active:
            self.tracking()
    
    # Funktion zur Zielnavigationssteuerung des Roboters
    def handle_navigation_command(self, ziel_name):
        if ziel_name in self.waypoints:
            ziel = self.waypoints.get(ziel_name)

            if ziel:
                x ,y , yaw = ziel
            self.navigate_to_pose(x, y, yaw)
        

    # Funktion zum scannen der Umgebung und stoppen bei Hinderniserkennung
    # Neuer Aufruf sobald neuer LIDAR Scan empfangen wurde
    def scan_callback(self, msg):

        self.latest_scan = msg # Abspeichern Scan für yolo_callback

        num_ranges = len(msg.ranges)

        front_center = num_ranges   # Beobachtet bei 360 Grad, sprich vorne
        window_front = msg.ranges[max(0, front_center - Angle):min(num_ranges, front_center + Angle)] # Beobachtet 360 - Angle : 359 + Angle Werte

        back_center = num_ranges // 2   # Beobachtet bei 360 Grad/2 = 180 Grad, sprich hinten
        window_back = msg.ranges[max(0, back_center - Angle):min(num_ranges, back_center + Angle)]

        left_center = num_ranges // 4  # Beobachtet bei 360 Grad/4, sprich 90° links
        window_left = msg.ranges[max(0, left_center - Angle):min(num_ranges, left_center + Angle)] # Beobachtet 360 - Angle : 359 + Angle Werte

        right_center = (num_ranges // 4) * 3  # Beobachtet bei 360 Grad/4, sprich 90° links
        window_right = msg.ranges[max(0, right_center - Angle):min(num_ranges, right_center + Angle)] # Beobachtet 360 - Angle : 359 + Angle Werte

        valid_ranges_front = [r for r in window_front if np.isfinite(r) and r > 0.05]   # Gültige Werte für Hinderniserkennung auf der Vorderseite
        valid_ranges_back = [r for r in window_back if np.isfinite(r) and r > 0.05]     # Gültige Werte für Hinderniserkennung auf der Rückseite
        valid_ranges_left = [r for r in window_left if np.isfinite(r) and r > 0.05]     # Gültige Werte für Hinderniserkennung links
        valid_ranges_right = [r for r in window_right if np.isfinite(r) and r > 0.05]     # Gültige Werte für Hinderniserkennung rechts

        if not valid_ranges_front or not valid_ranges_back or not valid_ranges_left or not valid_ranges_right:     # Beenden falls keine gültigen Werte erkannt wurden
            return

        self.min_distance_front = min(valid_ranges_front)            # kleinste Distanz der gemessenen Werte vorne
        self.min_distance_back = min(valid_ranges_back)              # kleinste Distanz der gemessenen Werte hinten
        self.min_distance_left = min(valid_ranges_left)
        self.min_distance_right = min(valid_ranges_right)


    #NEW BLOCK FOR YOLO INTEGRATION
    def yolo_callback(self, msg: String):
        text = msg.data.lower()
        if "door" in text or "tür" in text:
            self.get_logger().info("\nTür erkannt\n")
            if not self.latest_scan:
                return
            # Abstand inkl. Sicherheit abziehen
            raw_dist = self._get_front_distance(self.latest_scan)
            dist = max(0.0, raw_dist - Abstand)  # Sicherheitsabstand abziehen
            if not np.isfinite(dist):
                return
            x,y = self._calc_global_position(dist)
            if x is None or y is None:  #x und y auf Gültigkeit überprüfen
                return
            # Prüfen, ob Position schon in waypoints existiert
            for (vx,vy,yaw) in self.waypoints.values():
                if math.hypot(vx-x, vy-y) < Abstand:
                    self.get_logger().info("\nTür bereits bekannt\n")
                    return  # bereits bekannt
            # Neue Tür anlegen
            name = f"tür {len(self.waypoints)+1}"
            yaw = self._get_current_yaw()
            self.waypoints[name] = (x,y,yaw)
            self.get_logger().info(f"\nNeue Tür erkannt: {name} at ({x:.2f},{y:.2f})\n")


    #Bestimme die Distanz nach vorne heraus
    def _get_front_distance(self, scan: LaserScan) -> float:
        mid = len(scan.ranges)//2
        window = scan.ranges[mid-Angle_doorscan:mid+Angle_doorscan]     # Angle_doorscan als Winkel in welchem die geringste Distanz zur Tür bestimmt wird
        valid = [r for r in window if np.isfinite(r) and r>0]
        return min(valid) if valid else float('nan')


    #Bestimme die momentane Position des Roboters in der Karte
    def _calc_global_position(self, dist: float):
        pose = self.get_robot_pose_map()
        if pose is None:
            return None, None
        x0, y0, yaw = pose
        self.current_pose = pose
        return x0 + dist * math.cos(yaw), y0 + dist * math.sin(yaw)


    #Bestimme momentane Orientierung im Raum bzw. in der Karte
    def _get_current_yaw(self):
        pose = self.get_robot_pose_map()
        if pose is None:
            return None
        return pose[2]  # yaw


    #Berechne die Position aus map und baselink
    def get_robot_pose_map(self):
        try:
            #now = rclpy.time.Time()
            now = Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
            # Position
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            # Orientierung (Quaternion → Yaw)
            q = trans.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(f"Fehler beim transformieren: {e}")
            return None

    #END OF YOLO INTEGRATION


    # Funktion zur Zielübergabe an NavigateToPose
    def navigate_to_pose(self, x, y, yaw_rad):
  
        q = self.euler_to_quaternion(0, 0, yaw_rad)

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation = q

        self.get_logger().info(f"\nNavigiere zu: x={x}, y={y}, yaw={yaw_rad:.2f} rad\n")

        self.navigating = True
        self.navigator.goToPose(goal_pose)

        # Warten bis Navigation abgeschlossen ist
        while not self.navigator.isTaskComplete() and self.navigating == True:
                rclpy.spin_once(self, timeout_sec=0.1)

        if self.navigating:             
            result = self.navigator.getResult() 
        if result == TaskResult.SUCCEEDED:
                self.get_logger().info("✅ Ziel erfolgreich erreicht.")
                self.working = False
                self.navigating = False
                self.play_sound(1)
        elif result == TaskResult.FAILED:
                self.get_logger().warn("❌ Navigation fehlgeschlagen.")
                self.working = False
                self.navigating = False
                self.play_sound(2)
        elif result == TaskResult.CANCELED:
                self.get_logger().warn("⚠️ Navigation wurde abgebrochen.")
                self.working = False
                self.navigating = False

        self.navigator.cancelTask()
        self.navigating = False
        self.working = False
        self.get_logger().info("\n-----Warte auf neuen Befehl-----\n")
        self.output_commands()

        
    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Quaternion:
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - \
             math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + \
             math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)

        quat = Quaternion()
        quat.x = qx
        quat.y = qy
        quat.z = qz
        quat.w = qw
        return quat


def main():
    rclpy.init()
    node = YoloDetector()
    # Starte Command-Thread
    threading.Thread(target=node.command_loop, daemon=True).start()
    # ROS-Loop für alle Timer- und Topic-Callbacks
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

"""
def main(args=None):
    rclpy.init()
    node = YoloDetector()
    # Starte Eingabe-Thread
    threading.Thread(target=node.command_loop, daemon=True).start()
    rclpy.spin_once(node, timeout_sec=0.1)
    try:
        rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Node beendet')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    """
