import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan           # Für Hinderniserkennung
from rclpy.qos import qos_profile_sensor_data
from enum import Enum

import sounddevice as sd
import queue
import json
import vosk
import numpy as np

# Je nach dem wer den Code gerade testen will, ändert seinen Namen auf True und den anderen auf False für den richtigen Modelpfad
Andy = True
Bastian = False

Abstand = 0.3   # Abstand in Metern, bei dem ein Hindernis erkannt wird

class Hinderniserkennung(Enum):
    front = 1
    back = 2
    none = 3

class DirectionState(Enum):
    forward = 1
    backward = 2 
    none = 3

# Erstellen der Node
class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Pfad zum Vosk-Modell anpassen
        if Andy:
         model_path = r"/home/andy/Turtelbot3_voicecontroll/vosk-model-small-de-0.15"            #modelpath Andy
        if Bastian:
         model_path = r"/home/basti/Schreibtisch/Turtlebot/vosk-model-de-0.15"                   #modelpath Bastian

        self.model = vosk.Model(model_path)

        self.Hindernisserkennung = Hinderniserkennung.none
        self.DirectionState = DirectionState.none

        self.q = queue.Queue()
        self.twist = Twist()
        self.device_id = None

        self.stream = sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                                        channels=1, callback=self.audio_callback,
                                        device=self.device_id)
        self.stream.start()

        self.rec = vosk.KaldiRecognizer(self.model, 16000)

        self.get_logger().info("\n-----Sprachsteuerung gestartet-----\n Mögliche Befehle: vorwärts, zurück, halt, links, rechts, turn\n")

        self.timer = self.create_timer(0.1, self.timer_callback)

        # Hinderniserkennung
        self.obstacle_detected = False
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

#-----------------------------------------------------------------------------------------------
    #Funktion Ausgabe welches Audiogerät erkannt
    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Sounddevice Status: {status}")
        self.q.put(bytes(indata))

#-----------------------------------------------------------------------------------------------
    #Funktion Kommando erkennung
    def timer_callback(self):
        while not self.q.empty():
            data = self.q.get()
            if self.rec.AcceptWaveform(data):
                result = json.loads(self.rec.Result())
                text = result.get("text", "")
                if text:
                    self.get_logger().info(f"Erkannt: {text}")
                    self.handle_movement_Command(text)
#-----------------------------------------------------------------------------------------------
    #Funktion zur Bewegungssteuerung des Roboters
    def handle_movement_Command(self, text):
        # Bewegung zurücksetzen
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        if "vorwärts" in text:
            self.DirectionState = DirectionState.forward
            self.twist.linear.x = 0.2
        elif "zurück" in text:
            self.DirectionState = DirectionState.backward
            self.twist.linear.x = -0.2
        elif "links" in text:
            self.twist.angular.z = 0.2
        elif "rechts" in text:
            self.twist.angular.z = -0.2
        elif "turn" in text:
            self.twist.linear.x = 0.3
            self.twist.angular.z = -0.6
        elif "halt" in text:
            self.get_logger().info(f"Hält an\n-----Warte auf neuen Sprachbefehl-----\n")
        else:
            return  # Unbekannter Befehl
        
        self.pub.publish(self.twist)
#-----------------------------------------------------------------------------------------------
    #Funktion zum scannen der Umgebung und stoppen bei Hinderniserkennung
    #Neuer Aufruf sobal neuer LIDAR Scan empfangen wurde
    def scan_callback(self, msg):
       
        Angle = 20                      # gescannter Winkel in Grad
        num_ranges = len(msg.ranges)

        front_center = num_ranges   # Beobachtet bei 360 grad, sprich Vorne
        window_front = msg.ranges[max(0, front_center - Angle):min(num_ranges, front_center + Angle)] # Beobachtet 360 - Angle : 359 + Angle Werte

        back_center = num_ranges // 2 # Beobachtet bei 360 grad/2 = 180 grad, sprich hinten
        window_back = msg.ranges[max(0, back_center - Angle):min(num_ranges, back_center + Angle)]

        # Nur gültige Werte verwenden, werden in valid_ranges gespeichert
        # Rauschwellen um die 0.05m werden nicht berücksichtigt
        Frontwindow = list(window_front)
        Backwindow = list(window_back)

        valid_ranges_front = [r for r in Frontwindow if np.isfinite(r) and r > 0.05]        # Gültige Werte für Hindernisse auf der Vorderseite
        valid_ranges_back = [r for r in Backwindow if np.isfinite(r) and r > 0.05]          # Gültige Werte für Hindernisse auf der Rückseite

        if not valid_ranges_front or not valid_ranges_back: # Beenden falls keine gültigen Werte erkannt wurden
            return

        min_distance_front = min(valid_ranges_front)        # kleinste Distanz der gemessenen Werte vorne
        min_distance_back = min(valid_ranges_back)          # kleinste Distanz der gemessenen Werte hinten

        # Status umschalten nach Hinderniserkennung
        if min_distance_back <= Abstand:
            self.Hindernisserkennung = Hinderniserkennung.back

        elif min_distance_front <= Abstand:
            self.Hindernisserkennung = Hinderniserkennung.front

        else:
            if self.Hindernisserkennung != Hinderniserkennung.none:          # Umschalten auf Normalzustand
                self.get_logger().info("\n\nKein Hindernis mehr im Weg\n")
                self.Hindernisserkennung = Hinderniserkennung.none
                self.DirectionState = DirectionState.none
                stopTwist = Twist()
                self.pub.publish(stopTwist)

        # Hindernis wurde erkannt und Roboter befand sich in der Bewegung
        if self.Hindernisserkennung == Hinderniserkennung.front and self.DirectionState == DirectionState.forward:
            stopTwist = Twist()
            self.pub.publish(stopTwist)
            self.get_logger().warn(f"\n\n!!!!Hindernis Vorne erkannt in {min_distance_front:.2f} m – Hält an!\n")
            self.twist.linear.x = -0.2
            self.get_logger().info("\n\nRückwärts fahren bis kein Hindernis mehr im Weg\n")
            self.pub.publish(self.twist)

        elif self.Hindernisserkennung == Hinderniserkennung.back and self.DirectionState == DirectionState.backward:
            stopTwist= Twist()
            self.pub.publish(stopTwist)
            self.get_logger().warn(f"\n\n!!!!Hindernis Hinten erkannt in {min_distance_back:.2f} m – Hält an!\n")
            self.twist.linear.x = 0.2
            self.get_logger().info("\n\nVorwärts fahren bis kein Hindernis mehr im Weg\n")
            self.pub.publish(self.twist)


        
>>>>>>> feature
def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stream.stop()
        node.stream.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
