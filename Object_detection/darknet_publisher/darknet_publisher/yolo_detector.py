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


# Variablen für Funktionen
Abstand = 0.5                               # Abstand in Metern, bei dem ein Hindernis erkannt wird
Timer_callback_Aufrufsintervall = 0.1
Angle = 10                                  # gescannter Winkel in Grad
Angle_doorscan = 2                          # gescannter Winkel für Türerkennung
tracking_radius = 1                         #radius für start/endposition wand tracking
MenuTime = 5.0                              #Wie oft wird das Menü neu ausgegeben


class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

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

        self.sound_client = self.create_client(Sound, '/sound')

        self.input_timer = self.create_timer(Timer_callback_Aufrufsintervall, self.timer_callback)
        self.tracking_timer = self.create_timer(Timer_callback_Aufrufsintervall, self.tracking_loop)

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

        self.current_pose: Optional[Tuple[float, float, float]] = None
        self.start_pose: Optional[Tuple[float, float, float]] = None    #start_pose initialisieren als None

        self.tracking_active = False
        self.navigating = False
        self.working = False    #Wird gerade ein Befehl ausgeführt? Wenn ja, dann kein neues Kommando, muss erst bei Abschließen der Funktion wieder Freigegeben werden 
                                #(tracking zurück bei startpunkt, navigation bei position angekommen oder abgebrochen)

        # Hinderniserkennung
        self.obstacle_detected = False                  # Zur erkennung ob Hindernis erkannt wurde
        self.obstacle_handling_active = False           # Hinderniserkennungs Handling Status

        self.cmd_queue = queue.Queue()
        threading.Thread(target=self._input_loop, daemon=True).start()

        self.get_logger().info('Objekterkennung gestartet...')
        #self.timer_callback()


    def __del__(self):
        self.get_logger().info("YoloDetector wird zerstört!")


    #NEU EINGEFÜGT
    def _input_loop(self):
        #Einziger Thread, der input() liest und Befehle in eine Queue legt.
        while rclpy.ok():
            try:
                self.output_commands()
                cmd = input("Befehl: ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break
            if cmd:
                self.cmd_queue.put(cmd)


    def _tracking_worker(self):
        try:
            while self.tracking_active and rclpy.ok():
                # rufe die nicht-blockierende tracking() auf, die einmalig arbeitet
                self.tracking()
                # kurze Pause, damit CPU nicht 100%
                time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f"Tracking-Worker Fehler: {e}")
        finally:
            self.tracking_active = False
            self.working = False
    ###############


    #NEU EINGEFÜGT
    def timer_callback(self):
        # Menü nur zeigen ab und zu (throttle)
        self.output_commands()

        # Wenn bereits in Navigation/Tracking: nichts neues starten
        if self.working:
            return

        # Versuche einen Command aus der Queue (non-blocking)
        try:
            cmd = self.cmd_queue.get_nowait()
        except queue.Empty:
            return

        # Jetzt verarbeite den Befehl SERIELL IM HAUPTTREAD
        if cmd == "wand tracking":
            self.tracking_active = True
            self.working = True
            threading.Thread(target=self._tracking_worker, daemon=True).start()
            self.get_logger().info("Starte Wand-Tracking (Worker Thread)")
        elif cmd in self.waypoints:
            # Set flags and call navigate_to_pose directly (kein Worker-Thread!)
            self.working = True
            self.navigating = True
            x, y, yaw = self.waypoints.get(cmd)
            self.get_logger().info(f"Starte Navigation zu {cmd} (seriell im Hauptthread)")
            try:
                # navigate_to_pose blockiert, ruft intern rclpy.spin_once, ist aber safe hier
                self.navigate_to_pose(x, y, yaw)
            except Exception as e:
                self.get_logger().error(f"Fehler während Navigation: {e}")
            finally:
                self.navigating = False
                self.working = False
        else:
            self.get_logger().warn(f"Ungültiger Befehl: {cmd}")
    ####################


    def play_sound(self, sound_value=2):
        request = Sound.Request()
        request.value = sound_value
        future = self.sound_client.call_async(request)


    #NEU EINGEFÜGT
    def output_commands(self):
        #Zeigt die gültigen Befehle, aber nicht bei jeder Timer-Iteration (throttled).
        now = time.time()
        # last menu print timestamp (erstellt bei Bedarf)
        if not hasattr(self, '_last_menu_print'):
            self._last_menu_print = 0.0

        #das Menü ausgeben alle MenuTime sekunden (z.B. 3.5)
        if now - self._last_menu_print < MenuTime:
            return
        self._last_menu_print = now

        self.get_logger().info("    Gültige Befehle:")
        self.get_logger().info("Wand tracking")
        for i, name in enumerate(self.waypoints.keys(), start=1):
            self.get_logger().info(f"{i}: {name}")


    def command_read(self):
        while rclpy.ok() :
            if not self.working :
                cmd = input("Befehl: ").strip().lower()
                return cmd
            else : 
                self.get_logger().info("Befehl wird abgearbeitet")


    def tracking(self):
        self.get_logger().info("tracking start")

        twist = Twist()

        if self.current_pose is None:
            self.get_logger().info("Keine Gültige momentan Position")
            #warte bis Bedingung erfüllt
            #self.tracking_active = False
            #self.working = False
            return

        # --- 1. Startpunkt merken ---
        """
        while self.start_pose is None :
            if self.min_distance_front <= Abstand:
                self.get_logger().info("tracking start position setzen")
                self.start_pose = self.current_pose
                self.get_logger().info(f"Start-Pose gesetzt: x={self.start_pose[0]:.2f}, y={self.start_pose[1]:.2f}")
            else :
                self.get_logger().info("Zu weit weg von Wand für startpunkt")
                twist.linear.x = 0.2
        """
        #NEU EINGEFÜGT
        if self.start_pose is None:
            if self.min_distance_front <= Abstand:
                self.start_pose = self.current_pose
                self.get_logger().info(f"Start-Pose gesetzt: x={self.start_pose[0]:.2f}, y={self.start_pose[1]:.2f}")
                return
            else:
                self.get_logger().info("Zu weit weg von Wand für startpunkt")
                twist.linear.x = 0.2
                self.pub.publish(twist)
                return
        ################


        # --- 2. Prüfen, ob Startpunkt wieder erreicht wurde ---
        dx = self.current_pose[0] - self.start_pose[0]
        dy = self.current_pose[1] - self.start_pose[1]
        if math.hypot(dx, dy) <= tracking_radius:
            self.get_logger().info("Zurück am Startpunkt. Tracking endet.")
            self.start_pose = None
            self.tracking_active = False
            self.working = False
            self.play_sound(1)
            #self.command_queue.task_done()
            self.timer_callback()
            #self.command_queue = queue.Queue()
            return

        # --- 3. Hindernis direkt vorne: Nur nach rechts drehen, bis Front und Links frei sind ---
        if self.min_distance_front < Abstand or self.min_distance_left < Abstand:
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # Rechtsdrehung
            self.pub.publish(twist)
            return

        # --- 4. Normaler Wall-Following: PD-Regelung auf linken Abstand ---
        error = self.min_distance_left - Abstand
        kP = 1.0

        twist.linear.x = 0.2
        twist.angular.z = -kP * error
        self.pub.publish(twist)
        #self.tracking_loop()


    #Funktion zum wiederholten Aufruf von tracking
    def tracking_loop(self):
        #self.get_logger().info("tracking_loop start")
        if self.working and self.tracking_active:
            #self.get_logger().info("tracking active")
            self.tracking()
        else :
            #self.get_logger().info("tracking inactive")
            self. working = False
            self.tracking_active = False
        #self.get_logger().info("tracking_loop beendet")
        

    # Funktion zur Zielnavigationssteuerung des Roboters
    def handle_navigation_command(self, ziel_name):
        if ziel_name in self.waypoints:
            ziel = self.waypoints.get(ziel_name)
            if ziel:
                x ,y , yaw = ziel
            self.navigate_to_pose(
                x, # type: ignore
                y, # type: ignore 
                yaw # type: ignore
            )
        

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
        if "door" in text :
            if not self.latest_scan:
                return
            # Abstand inkl. Sicherheit abziehen
            raw_dist = self._get_front_distance(self.latest_scan)
            dist = max(0.0, raw_dist - Abstand)  # Sicherheitsabstand abziehen
            if not np.isfinite(dist):
                self.get_logger().info("\nTür zu weit entfernt\n")
                return
            x,y = self._calc_global_position(dist)
            if x is None or y is None:  #x und y auf Gültigkeit überprüfen
                return
            # Prüfen, ob Position schon in waypoints existiert
            for (vx,vy,yaw) in self.waypoints.values():
                if math.hypot(vx-x, vy-y) < Abstand:
                    self.get_logger().info(f"Tür bekannt")
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

    #Falls amcl keine Pose veröffentlicht hat, versuche eine Pose über den tf_tree zu berechnen
    def get_robot_pose_map(self):
        if self.current_pose is not None:
            return self.current_pose
        # Fallback: versuche TF lookup (optional)
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_footprint', now)
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            q = trans.transform.rotation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return (x, y, yaw)
        except Exception as e:
            self.get_logger().warn(f"Fehler beim transformieren: {e}")
            return None


    def amcl_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.get_logger().info("amcl_pose_callback start")

        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        q = msg.pose.pose.orientation
        # Quaternion -> yaw
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.current_pose = (x, y, yaw)
        self.get_logger().info(f"amcl_pose_callback: current_pose gesetzt: {self.current_pose}")
            

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

        self.navigator.cancelTask()
        self.navigating = False
        self.working = False
        self.get_logger().info("\n-----Warte auf neuen Befehl-----\n")
        self.timer_callback()

        
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