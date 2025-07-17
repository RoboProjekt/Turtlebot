import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore
from sensor_msgs.msg import LaserScan  # Für Hinderniserkennung # type: ignore
from turtlebot3_msgs.srv import Sound #type: ignore
from rclpy.qos import qos_profile_sensor_data  # type: ignore
from enum import Enum
from std_srvs.srv import SetBool

import sounddevice as sd  # type: ignore
import queue
import json
import vosk  # type: ignore
import numpy as np  # type: ignore
import math
import time
import threading

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult  # type: ignore
from geometry_msgs.msg import PoseStamped  # type: ignore
from geometry_msgs.msg import Quaternion  # type: ignore

# Eingragen wer den Code gerade benutzt
User = "pi"                               # andy oder bastian
samplerate_number = 16000                   # 16000 für NUtzung auf pi sonst 44100
blocksize_number = 4096
Abstand = 0.3                               # Abstand in Metern, bei dem ein Hindernis erkannt wird
Timer_callback_Aufrufsintervall = 0.01
Angle = 20                                  # gescannter Winkel in Grad

class Hinderniserkennung(Enum):
    front = 1
    back = 2
    none = 3

class DirectionState(Enum):
    forward = 1
    backward = 2
    circle = 3
    none = 4

# Erlaubte Befehle
Valid_Commands = {"zurück", "vorwärts", "links", "rechts", "kreis", "halt"}
Valid_point_Commands = {"tür flur", "tür labor", "stellplatz"}


Ausgabe_Befehlsliste = "\nMögliche Befehle: vorwärts, zurück, halt, links, rechts, kreis\n"
Ausagbe_Navigationsbefehle = "Mögliche Navigationsziele: Tür Flur, Tür Labor, Stellplatz\n"



# Erstellen der Node
class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Pfad zum Vosk-Modell, User oben im Code eintragen !!!!
        if User == "andy":
            model_path = r"/home/andy/Turtelbot3_voicecontroll/vosk-model-small-de-0.15"
        elif User == "bastian":
            model_path = r"/home/basti/Schreibtisch/Turtlebot/Voicecontrol/vosk-model-small-de-0.15"
        elif User == "pi":
            model_path =r"/home/pi/Git_Turtlebot/Turtlebot/Voicecontrol/vosk-model-small-de-0.15"

        def __del__(self):
            self.get_logger().info("VoiceControlNode wird zerstört!")

        self.model = vosk.Model(model_path)
        self.navigator = BasicNavigator()

        self.navigator.waitUntilNav2Active()

        self.navigating = False

        self.waypoints_list = {
            "tür flur": (1.25, 3.9),
            "tür labor": (-6.1, -0.95),
            "stellplatz": (-0.9, -1.8)
        }
        
        # Grad in Radiant für orientation_list
        # 0°   -> 0.00
        # 90°  -> 1.57
        # 180° -> 3.14
        # 270° -> 4.71
        self.orientation_list = {
            "tür flur": 1.57,   # 90° in Radiant
            "tür labor": 0.0,
            "stellplatz": 0.0
        }

        # Initialisieren der Zustände
        self.Hindernisserkennung = Hinderniserkennung.none
        self.DirectionState = DirectionState.none

        self.q = queue.Queue()
        self.twist = Twist()
        #Änder der Mikrophon ID notendig falls sie nicht auf Standart steht --> None entspricht Standart
        self.device_id = None

        self.stream = sd.RawInputStream(
            samplerate=samplerate_number, blocksize=blocksize_number, dtype='int16',
            channels=1, callback=self.audio_callback,
            device=self.device_id)
        self.stream.start()

        self.rec = vosk.KaldiRecognizer(self.model, 16000)

        self.get_logger().info("\n-----Sprachsteuerung gestartet-----\n")
        self.get_logger().info(Ausgabe_Befehlsliste)
        self.get_logger().info(Ausagbe_Navigationsbefehle)

        self.timer = self.create_timer(Timer_callback_Aufrufsintervall, self.timer_callback)

        # Hinderniserkennung
        self.obstacle_detected = False                  # Zur erkennung ob Hindernis erkannt wurde
        self.obstacle_handling_active = False           # Hinderniserkennungs Handling Status

        self.scan_sub = self.create_subscription(       # LIDAR Scan
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

    def set_motor_power(self, active: bool):

    client = self.create_client(SetBool, '/motor_power')
    while not client.wait_for_service(timeout_sec=1.0):
        self.get_logger().warn('/motor_power service not available...')

    req = SetBool.Request()
    req.data = active
    future = client.call_async(req)

    if active:
        self.get_logger().info("✅ LDS- und Antriebsmotor aktiviert.")
    else:
        self.get_logger().info("🛑 LDS- und Antriebsmotor deaktiviert (Ruhezustand).")


    # Funktion Handle der Audioaufnahme und Fehleranzeige bei Audioübertragungsfehlern
    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Sounddevice Status: {status}")
        self.q.put(bytes(indata))

    # Funktion zur Kommando Erkennung, mit Sperrung der Erkennnung bei Hinderniserkennung
    def timer_callback(self):
        while not self.q.empty() and self.Hindernisserkennung == Hinderniserkennung.none:
            data = self.q.get()
            if self.rec.AcceptWaveform(data):
                result = json.loads(self.rec.Result())
                command = result.get("text", "")
                if command in Valid_Commands:
                    self.get_logger().info(f"Gültiger Befehl erkannt: {command}")
                    self.set_motor_power(True)
                    self.handle_movement_Command(command)
                elif command in Valid_point_Commands:
                    self.get_logger().info(f"Ziel Befehl erkannt: {command}")
                    self.navigating = True
                    self.set_motor_power(True)
                    self.handle_navigation_command(command)


    # Funktion zur dynamischen Sprachbewegungssteuerung des Roboters
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
            self.twist.angular.z = 0.3
        elif "rechts" in text:
            self.twist.angular.z = -0.3
        elif "kreis" in text:
            self.DirectionState = DirectionState.circle
            self.twist.linear.x = 0.3
            self.twist.angular.z = -0.6
        elif "halt" in text:
            self.get_logger().info("\n-----Warte auf neuen Sprachbefehl-----\n")
            self.get_logger().info(Ausgabe_Befehlsliste)
            self.get_logger().info(Ausagbe_Navigationsbefehle)
            self.set_motor_power(False)
        else:
            return
        self.pub.publish(self.twist)        # Publishen des Befehls

    # Funktion zur Zielnavigationssteuerung des Roboters
    def handle_navigation_command(self, ziel_name):
        coords = self.waypoints_list.get(ziel_name)
        orientation = self.orientation_list.get(ziel_name, 0.0)

        if coords:
            x, y = coords
            self.navigate_to_pose(x, y, orientation)

    # Funktion zum scannen der Umgebung und stoppen bei Hinderniserkennung
    # Neuer Aufruf sobald neuer LIDAR Scan empfangen wurde
    def scan_callback(self, msg):
        num_ranges = len(msg.ranges)

        front_center = num_ranges   # Beobachtet bei 360 Grad, sprich vorne
        window_front = msg.ranges[max(0, front_center - Angle):min(num_ranges, front_center + Angle)] # Beobachtet 360 - Angle : 359 + Angle Werte

        back_center = num_ranges // 2   # Beobachtet bei 360 Grad/2 = 180 Grad, sprich hinten
        window_back = msg.ranges[max(0, back_center - Angle):min(num_ranges, back_center + Angle)]

        valid_ranges_front = [r for r in window_front if np.isfinite(r) and r > 0.05]   # Gültige Werte für Hinderniserkennung auf der Vorderseite
        valid_ranges_back = [r for r in window_back if np.isfinite(r) and r > 0.05]     # Gültige Werte für Hinderniserkennung auf der Rückseite

        if not valid_ranges_front or not valid_ranges_back:     # Beenden falls keine gültigen Werte erkannt wurden
            return

        min_distance_front = min(valid_ranges_front)            # kleinste Distanz der gemessenen Werte vorne
        min_distance_back = min(valid_ranges_back)              # kleinste Distanz der gemessenen Werte hinten

        # Status umschalten nach Hinderniserkennung
        if min_distance_back <= Abstand:
            self.Hindernisserkennung = Hinderniserkennung.back
        elif min_distance_front <= Abstand:
            self.Hindernisserkennung = Hinderniserkennung.front
        else:
            if self.Hindernisserkennung != Hinderniserkennung.none:         # Umschalten auf Normalzustand
                self.get_logger().info("\n\nKein Hindernis mehr im Weg\n")
                self.get_logger().info(Ausgabe_Befehlsliste)
                self.Hindernisserkennung = Hinderniserkennung.none
                self.DirectionState = DirectionState.none
                stopTwist = Twist()
                self.pub.publish(stopTwist)
                self.obstacle_handling_active = False
        # Hindernis wurde erkannt und Roboter befand sich in der Bewegung während der Erkennung
        if self.Hindernisserkennung == Hinderniserkennung.front and (self.DirectionState == DirectionState.forward or self.DirectionState == DirectionState.circle):
            stopTwist = Twist()
            self.pub.publish(stopTwist)
            self.twist.linear.x = -0.2
            self.pub.publish(self.twist)

            if not self.obstacle_handling_active:
                self.get_logger().warn(f"\n\n!!!!Hindernis Vorne erkannt in {min_distance_front:.2f} m  Hält an!\n")
                self.get_logger().info("\n\nRückwärts fahren bis kein Hindernis mehr im Weg\n")
                self.obstacle_handling_active = True

        elif self.Hindernisserkennung == Hinderniserkennung.back and (self.DirectionState == DirectionState.backward or self.DirectionState == DirectionState.circle):
            stopTwist = Twist()
            self.pub.publish(stopTwist)
            self.twist.linear.x = 0.2
            self.pub.publish(self.twist)

            if not self.obstacle_handling_active:
                self.get_logger().warn(f"\n\n!!!!Hindernis Hinten erkannt in {min_distance_back:.2f} m  Hält an!\n")
                self.get_logger().info("\n\nVorwärts fahren bis kein Hindernis mehr im Weg\n")
                self.obstacle_handling_active = True

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

        self.navigator.goToPose(goal_pose)

        # Warten bis Navigation abgeschlossen ist
        while not self.navigator.isTaskComplete() and self.navigating == True:
                time.sleep(0.1)

        if self.navigating:             
            result = self.navigator.getResult() 
        if result == TaskResult.SUCCEEDED:
                self.get_logger().info("✅ Ziel erfolgreich erreicht.")
        elif result == TaskResult.FAILED:
                self.get_logger().warn("❌ Navigation fehlgeschlagen.")
        elif result == TaskResult.CANCELED:
                self.get_logger().warn("⚠️ Navigation wurde abgebrochen.")

        self.navigator.cancelTask()
        self.navigating = False
        self.get_logger().info("\n-----Warte auf neuen Sprachbefehl-----\n")
        self.get_logger().info(Ausgabe_Befehlsliste)
        self.get_logger().info(Ausagbe_Navigationsbefehle)
        self.set_motor_power(False)
        

        
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


