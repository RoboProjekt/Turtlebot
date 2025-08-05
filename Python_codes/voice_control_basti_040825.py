import rclpy  # type: ignore
from rclpy.node import Node  # type: ignore
from geometry_msgs.msg import Twist  # type: ignore
from sensor_msgs.msg import LaserScan  # Für Hinderniserkennung # type: ignore
from turtlebot3_msgs.srv import Sound #type: ignore
from rclpy.qos import qos_profile_sensor_data  # type: ignore
from enum import Enum
from std_srvs.srv import SetBool                #type:ignore

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

#NEW FOR YOLO
from std_msgs.msg import String  # type: ignore
#IMPORT MESSAGE STRING FOR YOLO CHECK
#Neu für current_position
from tf2_ros import Buffer, TransformListener #, LookupException, ConnectivityException, ExtrapolationException  ->Nur nötig bei Fehlerausgabe

# Eingragen wer den Code gerade benutzt
User = "pi"                               # andy oder bastian oder pi
samplerate_number = 16000                   
blocksize_number = 4096
Abstand = 0.3                               # Abstand in Metern, bei dem ein Hindernis erkannt wird
Timer_callback_Aufrufsintervall = 0.01
Angle = 20                                  # gescannter Winkel in Grad
Angle_doorscan = 2                          # gescannter Winkel für Türerkennung

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
Valid_point_Commands = {"eingang vorne", "eingang mitte", "eingang hinten", "stellplatz", "start", "flur anfang", "flur mitte", "flur ende", "eingang büro", "sofa"}


Ausgabe_Befehlsliste = "\nMögliche Befehle: vorwärts, zurück, halt, links, rechts, kreis\n"
Ausgabe_Navigationsbefehle = "Mögliche Navigationsziele: Eingang vorne, Eingang mitte , Eingang hinten, Stellplatz, Start, Flur Anfang, Flur mitte, Flur ende, Eingang büro, sofa\n"



# Erstellen der Node
class VoiceControlNodeBasti(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        #NEW INCLUDE YOLO OBJECT DETECTION
        self.sub_yolo = self.create_subscription(String, 'yolo_objects', self.yolo_callback, 10)

        #Neuer Subscriber für map und baselink zum Umrechnen in current_position
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        
        # Grad in Radiant für orientation_list
        # 0°   -> 0.00
        # 90°  -> 1.57
        # 180° -> 3.14
        # 270° -> 4.71
        """self.orientation_list = {
           # "flur": 1.57,   # 90° in Radiant
             "eingang vorne": 4.71 ,
             "eingang mitte": 4.71 ,
             "eingang hinten": 4.71 ,
             "stellplatz": 3.14 ,
             "start": 0.00 ,
             "flur anfang": 3.14 ,
             "flur mitte": 3.14 ,
             "flur ende": 0.00 ,
             "eingang büro": 3.14 ,
             "sofa": 0.00

        }
        """

        #NEW SCAN FOR DOOR LOCATION
        self.latest_scan = None

        # Pfad zum Vosk-Modell, User oben im Code eintragen !!!!
        if User == "andy":
            model_path = r"/home/andy/Turtelbot3_voicecontroll/vosk-model-small-de-0.15"
        elif User == "bastian":
            model_path = r"/home/basti/Schreibtisch/Turtlebot/Python_codes/vosk-model-small-de-0.15"
        elif User == "pi":
            model_path =r"/home/pi/Git_Turtlebot/Turtlebot/Python_codes/vosk-model-small-de-0.15"

        """def __del__(self):
            self.get_logger().info("VoiceControlNode wird zerstört!")
        """

        self.model = vosk.Model(model_path)
        self.navigator = BasicNavigator()

        self.navigator.waitUntilNav2Active()

        self.navigating = False

        

        self.sound_client = self.create_client(Sound, '/sound')
        # Initialisieren der Zustände
        self.Hinderniserkennung = Hinderniserkennung.none
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
        self.get_logger().info(Ausgabe_Navigationsbefehle)

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

    def __del__(self):
        self.get_logger().info("VoiceControlNode wird zerstört!")

    def play_sound(self, sound_value=2):
        request = Sound.Request()
        request.value = sound_value
        future = self.sound_client.call_async(request)

    
    # Funktion Handle der Audioaufnahme und Fehleranzeige bei Audioübertragungsfehlern
    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Sounddevice Status: {status}")
        self.q.put(bytes(indata))

    # Funktion zur Kommando Erkennung, mit Sperrung der Erkennnung bei Hinderniserkennung
    def timer_callback(self):
        while not self.q.empty() and self.Hinderniserkennung == Hinderniserkennung.none:
            data = self.q.get()
            if self.rec.AcceptWaveform(data):
                result = json.loads(self.rec.Result())
                command = result.get("text", "")
                if command in Valid_Commands:
                    self.get_logger().info(f"Gültiger Befehl erkannt: {command}")
                    self.handle_movement_Command(command)
                elif command in Valid_point_Commands:
                    self.get_logger().info(f"Ziel Befehl erkannt: {command}")
                    self.navigating = True
                    self.play_sound(3)
                    self.handle_navigation_command(command)
                    #
                #NEW FOR DOOR LIST OF PREVIOUSLY UNKNOWN DOORS
             #   elif command.startswith("tür "):
             #       self.handle_navigation_command(command)
                


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
            self.get_logger().info(Ausgabe_Navigationsbefehle)
        else:
            return
        self.pub.publish(self.twist)        # Publishen des Befehls

    # Funktion zur Zielnavigationssteuerung des Roboters
    # Funktion zur Zielnavigationssteuerung des Roboters
    #NEW FUNCTION TO WORK WITH NEW LIST MANAGEMENT
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

        valid_ranges_front = [r for r in window_front if np.isfinite(r) and r > 0.05]   # Gültige Werte für Hinderniserkennung auf der Vorderseite
        valid_ranges_back = [r for r in window_back if np.isfinite(r) and r > 0.05]     # Gültige Werte für Hinderniserkennung auf der Rückseite

        if not valid_ranges_front or not valid_ranges_back:     # Beenden falls keine gültigen Werte erkannt wurden
            return

        min_distance_front = min(valid_ranges_front)            # kleinste Distanz der gemessenen Werte vorne
        min_distance_back = min(valid_ranges_back)              # kleinste Distanz der gemessenen Werte hinten

        # Status umschalten nach Hinderniserkennung
        if min_distance_back <= Abstand:
            self.Hinderniserkennung = Hinderniserkennung.back
        elif min_distance_front <= Abstand:
            self.Hinderniserkennung = Hinderniserkennung.front
        else:
            if self.Hinderniserkennung != Hinderniserkennung.none:         # Umschalten auf Normalzustand
                self.get_logger().info("\n\nKein Hindernis mehr im Weg\n")
                self.get_logger().info(Ausgabe_Befehlsliste)
                self.Hinderniserkennung = Hinderniserkennung.none
                self.DirectionState = DirectionState.none
                stopTwist = Twist()
                self.pub.publish(stopTwist)
                self.obstacle_handling_active = False
        # Hindernis wurde erkannt und Roboter befand sich in der Bewegung während der Erkennung
        if self.Hinderniserkennung == Hinderniserkennung.front and (self.DirectionState == DirectionState.forward or self.DirectionState == DirectionState.circle):
            stopTwist = Twist()
            self.pub.publish(stopTwist)
            self.twist.linear.x = -0.2
            self.pub.publish(self.twist)

            if not self.obstacle_handling_active:
                self.get_logger().warn(f"\n\n!!!!Hindernis Vorne erkannt in {min_distance_front:.2f} m  Hält an!\n")
                self.get_logger().info("\n\nRückwärts fahren bis kein Hindernis mehr im Weg\n")
                self.obstacle_handling_active = True

        elif self.Hinderniserkennung == Hinderniserkennung.back and (self.DirectionState == DirectionState.backward or self.DirectionState == DirectionState.circle):
            stopTwist = Twist()
            self.pub.publish(stopTwist)
            self.twist.linear.x = 0.2
            self.pub.publish(self.twist)

            if not self.obstacle_handling_active:
                self.get_logger().warn(f"\n\n!!!!Hindernis Hinten erkannt in {min_distance_back:.2f} m  Hält an!\n")
                self.get_logger().info("\n\nVorwärts fahren bis kein Hindernis mehr im Weg\n")
                self.obstacle_handling_active = True


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

    def _get_front_distance(self, scan: LaserScan) -> float:
        mid = len(scan.ranges)//2
        window = scan.ranges[mid-Angle_doorscan:mid+Angle_doorscan]     # Angle_doorscan als Winkel in welchem die geringste Distanz zur Tür bestimmt wird
        valid = [r for r in window if np.isfinite(r) and r>0]
        return min(valid) if valid else float('nan')
    """
    def _calc_global_position(self, dist: float):
        pose = self.get_robot_pose_map()
        q = pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        x0,y0 = pose.pose.position.x, pose.pose.position.y
        return x0 + dist*math.cos(yaw), y0 + dist*math.sin(yaw)
    """

    def _calc_global_position(self, dist: float):
        pose = self.get_robot_pose_map()
        if pose is None:
            return None, None
        x0, y0, yaw = pose
        return x0 + dist * math.cos(yaw), y0 + dist * math.sin(yaw)

    """
    def _get_current_yaw(self):
        pose = self.navigator.get_pose()
        q = pose.pose.orientation
        return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    """

    def _get_current_yaw(self):
        pose = self.get_robot_pose_map()
        if pose is None:
            return None
        return pose[2]  # yaw

    
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
        #except (LookupException, ConnectivityException, ExtrapolationException) as e:
            #self.get_logger().warn(f"❗ Fehler beim Abrufen der Roboterpose: {e}")
            #return None
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

        self.navigator.goToPose(goal_pose)

        # Warten bis Navigation abgeschlossen ist
        while not self.navigator.isTaskComplete() and self.navigating == True:
                time.sleep(0.8)

        if self.navigating:             
            result = self.navigator.getResult() 
        if result == TaskResult.SUCCEEDED:
                self.get_logger().info("✅ Ziel erfolgreich erreicht.")
                self.play_sound(1)
        elif result == TaskResult.FAILED:
                self.get_logger().warn("❌ Navigation fehlgeschlagen.")
                self.play_sound(2)
        elif result == TaskResult.CANCELED:
                self.get_logger().warn("⚠️ Navigation wurde abgebrochen.")

        self.navigator.cancelTask()
        self.navigating = False
        self.get_logger().info("\n-----Warte auf neuen Sprachbefehl-----\n")
        self.get_logger().info(Ausgabe_Befehlsliste)
        self.get_logger().info(Ausgabe_Navigationsbefehle)
        

        
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

