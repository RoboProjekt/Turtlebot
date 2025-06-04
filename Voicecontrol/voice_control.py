import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import sounddevice as sd
import queue
import json
import vosk
import sys

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

#Dateipfad angeben in dem vosk-model-small-de-0.15 gespeichert ist
        model_path = "<Datei/Pfrad/zu>/vosk-model-small-de-0.15"

        self.get_logger().info(f"Lade Vosk-Modell von: {model_path}")
        self.model = vosk.Model(model_path)

        self.q = queue.Queue()
        self.twist = Twist()

        # Mikrofon-Gerät ID hier anpassen, oder None für Standard
        self.device_id = 1

        self.stream = sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                                        channels=1, callback=self.audio_callback,
                                        device=self.device_id)
        self.stream.start()

        self.rec = vosk.KaldiRecognizer(self.model, 16000)

        self.get_logger().info("Sprachsteuerung gestartet - sag: vorwärts, rückwärts, links, rechts, halt")

        self.timer = self.create_timer(0.1, self.timer_callback)

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Sounddevice Status: {status}")
        self.q.put(bytes(indata))

    def timer_callback(self):
        while not self.q.empty():
            data = self.q.get()
            if self.rec.AcceptWaveform(data):
                result = json.loads(self.rec.Result())
                text = result.get("text", "")
                if text:
                    self.get_logger().info(f"Erkannt: {text}")
                    self.handle_command(text)

    def handle_command(self, text):
        # Reset Bewegung
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
        elif "kreis" in text:
            self.twist.linear.x = 0.2
            self.twist.angular.z = -0.2
        elif "halt" in text:
            # Keine Bewegung
            pass
        else:
            # Unbekannter Befehl, nichts tun
            return

        self.pub.publish(self.twist)
        self.get_logger().info(f"Bewegung gesendet: linear.x={self.twist.linear.x}, angular.z={self.twist.angular.z}")

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
