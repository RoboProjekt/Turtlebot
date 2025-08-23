# Neue Logging Ausgaben für debugging nach Nav2Pose (evntl hat return in nav2pose schon gefixt) -> Ansonsten working und navigating Überprüfen 
# Neue Variable für Tür anlegen ab gewisser sicherheit
# Umgestellte Positions und Orientationsbestimmung nach AMCL daten

import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from rclpy.time import Time # type: ignore
from rclpy.qos import qos_profile_sensor_data  # type: ignore

from geometry_msgs.msg import Twist  # type: ignore
from sensor_msgs.msg import LaserScan  # Für Hinderniserkennung # type: ignore
from turtlebot3_msgs.srv import Sound #type: ignore
from std_msgs.msg import String  # type: ignore
from geometry_msgs.msg import PoseStamped  # type: ignore
from geometry_msgs.msg import Quaternion  # type: ignore
from geometry_msgs.msg import PoseWithCovarianceStamped # type: ignore
from typing import Optional, Tuple

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult  # type: ignore
from tf2_ros import Buffer, TransformListener # type: ignore #, LookupException, ConnectivityException, ExtrapolationException  ->Nur nötig bei Fehlerausgabe

import sounddevice as sd  # type: ignore
import queue
import numpy as np  # type: ignore
import math
import time
import threading
import sys
import re


# Variablen für Funktionen
Abstand = 0.5                               # Abstand in Metern, bei dem auf bekannte Position geprüft wird
door_distance = 0.5                         #Sicherheitsabstand für Türerkennung
Timer_callback_Aufrufsintervall = 0.1       #Wie oft soll die Timer_callback ausgelöst werden?
Angle_doorscan = 2                          # gescannter Winkel für Türerkennung
tracking_radius = 1                         #radius für start/endposition wand tracking
MenuTime = 5.0                              #Wie oft wird das Menü neu ausgegeben
doorConf = 70                               #Wie sicher muss die Objekterkennung sein, damit die Türen verarbeitet werden


class YoloDetector(Node):
    def __init__(self):
        super().__init__('YoloDetector')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Objekterkennung gestartet...')
        #INCLUDE YOLO OBJECT DETECTION
        self.sub_yolo = self.create_subscription(
            String,
            'yolo_objects', 
            self.yolo_callback, 
            10
        )

        self.scan_sub = self.create_subscription(       # LIDAR Scan
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )

        self.timer = self.create_timer(Timer_callback_Aufrufsintervall, self.timer_callback)

        self.sound_client = self.create_client(Sound, '/sound')

        #Neuer Subscriber für map und baselink zum Umrechnen in current_position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()
        self.twist = Twist()

        self.waypoints = {
           # "flur": (1.25, 3.9),
             "eingang vorne": (18.5, 1.72, 4.71),
             "eingang mitte": (6.05, 1.77, 4.71),
             #"eingang hinten": (-1.7, 1.57, 4.71),
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

        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.start_pose: Optional[Tuple[float, float, float]] = None    #start_pose initialisieren als None

        self.tracking_active = False
        self.navigating = False
        self.working = False    #Wird gerade ein Befehl ausgeführt? Wenn ja, dann kein neues Kommando, muss erst bei Abschließen der Funktion wieder Freigegeben werden 
                                #(tracking zurück bei startpunkt, navigation bei position angekommen oder abgebrochen)

        # Hinderniserkennung
        self.obstacle_detected = False                  # Zur erkennung ob Hindernis erkannt wurde
        self.obstacle_handling_active = False           # Hinderniserkennungs Handling Status

        #NEU EINGEFÜGT
        self.cmd_queue = queue.Queue()
        # Start input thread (liest stdin nur hier)
        self.console_lock = threading.Lock()
        threading.Thread(target=self.input_loop, daemon=True).start()
        threading.Thread(target=self.navigate_to_pose, daemon=True).start()
        ###############


    def __del__(self):
        self.get_logger().info("YoloDetector wird zerstört!")


    def input_loop(self):
        while rclpy.ok() and self.working == False:
            try:
                # Prompt auf stderr (Logger nutzt stderr) und flush
                sys.stderr.write("Befehl: ")
                sys.stderr.flush()
                line = sys.stdin.readline()
                if not line:
                    break
                cmd = line.strip().lower()
                if cmd:
                    self.cmd_queue.put(cmd)
            except (EOFError, KeyboardInterrupt):
                break
    

    #NEU EINGEFÜGT
    def timer_callback(self):
        #self._input_loop()
        # Wenn bereits in Navigation/Tracking: nichts neues starten
        if not self.working and not self.navigating:
            self.output_commands()
            # Versuche einen Command aus der Queue (non-blocking)
            try:
                cmd = self.cmd_queue.get_nowait()
            except queue.Empty:
                return

            if cmd in self.waypoints:
                # Set flags and call navigate_to_pose directly (kein Worker-Thread!)
                self.working = True
                self.navigating = True
                x, y, yaw = self.waypoints.get(cmd)
                self.get_logger().info(f"Starte Navigation zu {cmd} (seriell im Hauptthread)")
                try:
                    self.get_logger().info(f"try vor nav2pose")
                    self.navigate_to_pose(x, y, yaw)
                    self.get_logger().info(f"try nach nav2pose")
                except Exception as e:
                    self.get_logger().error(f"Fehler während Navigation: {e}")
                finally:
                    self.get_logger().info(f"finally nach nav2pose")
                    self.navigating = False
                    self.working = False
                    self.get_logger().info(f"navigating und working auf false nach nav2pose")
            else:
                self.get_logger().warn(f"Ungültiger Befehl: {cmd}")
    ####################


    def play_sound(self, sound_value=2):
        request = Sound.Request()
        request.value = sound_value
        future = self.sound_client.call_async(request)


    #NEU EINGEFÜGT
    def output_commands(self):
        now = time.time()
        # last menu print timestamp (erstellt bei Bedarf)
        if not hasattr(self, '_last_menu_print'):
            self._last_menu_print = 0.0

        #das Menü ausgeben alle MenuTime sekunden (z.B. 3.5)
        if now - self._last_menu_print < MenuTime:
            return
        self._last_menu_print = now

        self.get_logger().info("Gültige Befehle:")
        for i, name in enumerate(self.waypoints.keys(), start=1):
            self.get_logger().info(f"{i}: {name}")


    def command_read(self):
        while rclpy.ok() :
            if not self.working :
                cmd = input("Befehl: ").strip().lower()
                return cmd
            else : 
                self.get_logger().info("Befehl wird abgearbeitet")


    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg  # für yolo_callback / spätere Projektion
        n = len(msg.ranges)
        if n == 0:
            return
        angle_min = msg.angle_min
        inc = msg.angle_increment
        # nutze als minimalen akzeptablen Range den Scan.range_min (plus ein kleines epsilon)
        min_acceptable = max(msg.range_min, 0.05)
        max_acceptable = msg.range_max
        # Hilfsfunktion: normalisieren in [-pi, pi]
        def _norm(a):
            return math.atan2(math.sin(a), math.cos(a))
        # gewünschte Richtungen (in base_footprint / robot-frame, 0 = vorwärts)
        desired_in_base = {
            "front": 0.0,
            "back": math.pi,
            "left": math.pi / 2.0,
            "right": -math.pi / 2.0,
        }
        # Anzahl Bins links/rechts vom Zentrum (wrap-aware)
        half_bins = max(1, int(round(math.radians(Angle_doorscan) / abs(inc))))  #Angle gibt das scan Fenster für alle Seiten vor
        results = {}  # name -> minimaler Abstand (oder inf)
        # Versuche TF: Ermittle Drehung (yaw) des scan-frames in base_footprint, falls möglich.
        scan_frame = msg.header.frame_id if hasattr(msg, "header") else None
        yaw_scan_in_base = 0.0
        try:
            if scan_frame and hasattr(self, "tf_buffer"):
                # Transformiere: wir holen die Pose von scan_frame in base_footprint
                now = rclpy.time.Time()
                trans = self.tf_buffer.lookup_transform('base_footprint', scan_frame, now)
                q = trans.transform.rotation
                yaw_scan_in_base = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
                # yaw_scan_in_base ist die Rotation (z) von scan_frame relativ zu base_footprint
        except Exception:
            # TF nicht verfügbar oder Fehler -> fallback: yaw_scan_in_base = 0 (angenommen aligned)
            yaw_scan_in_base = 0.0
        # Für jede Seite: bestimme das Fenster und den minimalen gültigen Wert
        for name, desired_base_angle in desired_in_base.items():
            # gewünschter Winkel in Scan-Frame (wenn scan nicht aligned ist, kompensieren)
            desired_scan_angle = _norm(desired_base_angle - yaw_scan_in_base)
            # Center-Index im Scan-Array (wrap)
            idx_center = int(round((desired_scan_angle - angle_min) / inc)) % n
            # Fenster-Indices (wrap-aware)
            indices = [(idx_center + i) % n for i in range(-half_bins, half_bins + 1)]
            # Filter gültiger Werte im Fenster
            valid = []
            for i in indices:
                r = msg.ranges[i]
                if np.isfinite(r) and (r >= min_acceptable) and (r <= max_acceptable):
                    valid.append((i, r))
            if not valid:
                results[name] = float('inf')  # kein gültiger Wert im Fenster
            else:
                abs_idx, raw_r = min(valid, key=lambda x: x[1])
                results[name] = raw_r
        # Wenn du exakt das ursprüngliche Verhalten behalten willst (Abbruch, falls irgendeine Seite fehlt),
        # dann kannst du wie vorher returnen. Ich empfehle stattdessen, nur zu returnen wenn *alle* Seiten fehlen.
        # Hier verhalte ich mich wie ursprünglich: abbrechen wenn eine Seite keine gültigen Messungen hat.
        if (not np.isfinite(results["front"])) or (not np.isfinite(results["back"])) or \
        (not np.isfinite(results["left"])) or (not np.isfinite(results["right"])):
            # Optional: logge welche Seite fehlt
            missing = [k for k, v in results.items() if not np.isfinite(v)]
            self.get_logger().debug(f"Scan window missing for sides: {missing}")
            return
        # Setze die minimalen Distanzen (wie vorher)
        self.min_distance_front = results["front"]
        self.min_distance_back = results["back"]
        self.min_distance_left = results["left"]
        self.min_distance_right = results["right"]


    #Immer wenn amcl eine Pose veröffentlicht wird die neue Position als aktuelle Roboterposition weggespeichert
    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):   
        self.get_logger().info("amcl_pose_callback start")
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        # Quaternion -> yaw
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.current_pose = (x, y, yaw)
        self.get_logger().info(f"amcl_pose_callback: current_pose gesetzt: {self.current_pose}")


    #NEW BLOCK FOR YOLO INTEGRATION
    def yolo_callback(self, msg: String):
        text = msg.data.lower()
        #self.new_door = False
        if "door" in text:
            match = re.search(r"(\d+)%", text)
            if match:
                zahl = int(match.group(1))
                if zahl > doorConf:
                    if not self.latest_scan:
                        return
                    # Abstand inkl. Sicherheit abziehen
                    #raw_dist = self._get_front_distance(self.latest_scan)
                    raw_dist = self.min_distance_front
                    dist = max(0.0, raw_dist - door_distance)  # Sicherheitsabstand abziehen
                    if not np.isfinite(dist):
                        self.get_logger().info("\nTür zu weit entfernt\n")
                        return
                    x,y = self._calc_global_position(dist)
                    if x is None or y is None:  #x und y auf Gültigkeit überprüfen
                        return
                    # Prüfen, ob Position schon in waypoints existiert
                    for (vx,vy,yaw) in self.waypoints.values():
                        if math.hypot(vx-x, vy-y) < Abstand:
                            self.get_logger().info(f"Tür bereits bekannt\n")
                            return  # bereits bekannt
                    #self.new_door = True
                    # Neue Tür anlegen
                    name = f"tür {len(self.waypoints)+1}"
                    yaw = self._get_current_yaw()
                    self.waypoints[name] = (x,y,yaw)
                    self.get_logger().info(f"\nRoboter Position: {self.current_pose}")
                    self.get_logger().info(f"Neue Tür erkannt: {name} at ({x:.2f},{y:.2f})\n")
        

    #Bestimme die momentane Position der Tür in der Karte
    def _calc_global_position(self, dist: float):
        pose = self.current_pose #Bestimme die momentane Position des Roboters in der Karte
        if pose is None:
            return None, None
        x0, y0, yaw = pose
        self.current_pose = pose
        return x0 + dist * math.cos(yaw), y0 + dist * math.sin(yaw) #Berechne Position der Tür in abhängigkeit von minimaler Distanz nach vorne


    #Bestimme momentane Orientierung im Raum bzw. in der Karte
    def _get_current_yaw(self):
        pose = self.current_pose
        if pose is None:
            return None
        return pose[2]  # yaw


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

        while not self.navigator.isTaskComplete() and self.navigating == True:
            time.sleep(0.1)

        if self.navigating:             
            result = self.navigator.getResult() 
            if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("✅ Ziel erfolgreich erreicht.")
                    self.current_pose = (x, y, yaw_rad)
                    self.navigating = False
                    self.play_sound(1)
            elif result == TaskResult.FAILED:
                    self.get_logger().warn("❌ Navigation fehlgeschlagen.")
                    self.navigating = False
                    self.play_sound(2)
            elif result == TaskResult.CANCELED:
                    self.get_logger().warn("⚠️ Navigation wurde abgebrochen.")
                    self.navigating = False

        self.get_logger().info(f"nav2pose vor cancelTask")
        self.navigator.cancelTask()
        self.get_logger().info(f"nav2pose nach cancelTask, vor navigating und working")
        self.navigating = False
        self.working = False
        self.get_logger().info(f"nav2pose nach navigating und working False")
        #self.get_logger().info("\n-----Warte auf neuen Befehl-----\n")
        return

        
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
