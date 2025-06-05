import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan           # Für Hinderniserkennung
from rclpy.qos import qos_profile_sensor_data

import sounddevice as sd
import queue
import json
import vosk
import numpy as np

Abstand = 0.3   # Abstand in Metern, bei dem ein Hindernis erkannt wird

# Erstellen der Node
class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Pfad zum Vosk-Modell anpassen
        model_path = r"/home/andy/Turtelbot3_voicecontroll/vosk-model-small-de-0.15"

        self.model = vosk.Model(model_path)

        self.q = queue.Queue()
        self.twist = Twist()
        self.device_id = None

        self.stream = sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                                        channels=1, callback=self.audio_callback,
                                        device=self.device_id)
        self.stream.start()

        self.rec = vosk.KaldiRecognizer(self.model, 16000)

        self.get_logger().info("\n-----Sprachsteuerung gestartet-----\n Mögliche Befehle: vorwärts, rückwärts, halt, links, rechts, fahr im kreis\n")

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
                    self.handle_command(text)
#-----------------------------------------------------------------------------------------------
    #Funktion um auf erhaltene Kommandos zu reagieren
    def handle_command(self, text):
        # Bewegung zurücksetzen
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        if "vorwärts" in text:
            self.twist.linear.x = 0.2
        elif "rückwärts" in text:
            self.twist.linear.x = -0.2
        elif "links" in text:
            self.twist.angular.z = 0.2
        elif "rechts" in text:
            self.twist.angular.z = -0.2
        elif "fahr im kreis" in text:
            self.twist.linear.x = 0.3
            self.twist.angular.z = -0.6
        elif "halt" in text:
            self.get_logger().info(f"Hält an\nWarte auf neuen Sprachbefehl\n")
        else:
            return  # Unbekannter Befehl

   
        self.pub.publish(self.twist)
#-----------------------------------------------------------------------------------------------
    #Funktion zum scannen der Umgebung und stoppen bei Hinderniserkennung
    #Neuer Aufruf sobal neuer LIDAR Scan empfangen wurde
    def scan_callback(self, msg):

        # Bereich in der Mitte wird beobachtet und ausgewertet (ca. ±10°) 
        center_index = len(msg.ranges) // 2
        window = msg.ranges[center_index - 10:center_index + 10]

        # Nur gültige Werte verwenden
        # Rauschwellen um die 0.05m werden nicht berücksichtigt
        valid_ranges = [r for r in window if np.isfinite(r) and r > 0.05]
        if not valid_ranges:
            return

        min_distance = min(valid_ranges)

        #Stoppen wenn Hindernis erkannt wurde, in vorgegebener Reichweite
        if min_distance < Abstand:
            if not self.obstacle_detected:
                self.get_logger().warn(f"\n\n!!!!Hindernis in {min_distance:.2f} m erkannt – Hält sofort an!!!!\n")
            self.obstacle_detected = True

            # Sofort stoppen bei Hinderniserkennung
            stop_twist = Twist()
            self.pub.publish(stop_twist)
            return
        else:
            self.obstacle_detected = False


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
