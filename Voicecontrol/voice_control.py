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

        # Hier Verzeichnis angeben, in welchem die vosk-model-small-de-0.15 abgelegt ist
        model_path = r"/home/andy/Turtelbot3_voicecontroll/vosk-model-small-de-0.15"

        self.get_logger().info(f"Lade Vosk-Modell von: {model_path}")
        self.model = vosk.Model(model_path)

        self.q = queue.Queue()
        self.twist = Twist()

        # Mikrofon-Gerät ID hier anpassen, oder None für Standard
        self.device_id = None

        self.stream = sd.RawInputStream(samplerate=16000, blocksize=8000, dtype='int16',
                                        channels=1, callback=self.audio_callback,
                                        device=self.device_id)
        self.stream.start()

        self.rec = vosk.KaldiRecognizer(self.model, 16000)

        self.get_logger().info("Sprachsteuerung gestartet \n- folgende Befehle werden unterstützt: vorwärts, fahr im kreis, rückwärts, links, rechts, halt")

        self.timer = self.create_timer(0.1, self.timer_callback)  # 0.1 Sekunden = 100 Millisekunden

    def audio_callback(self, indata, frames, time, status):
        if status:
            self.get_logger().warn(f"Sounddevice Status: {status}")
        self.q.put(bytes(indata))

# Funktion um Kommando zu empfangen, durch User
    def timer_callback(self):
        while not self.q.empty():
            data = self.q.get()
            if self.rec.AcceptWaveform(data):
                result = json.loads(self.rec.Result())
                text = result.get("text", "")
                if text:
                    if self.handle_command(text):
                        self.get_logger().info(f"Richtiger Befehl erkannt: {text}")
                    
                   
                    

#Funktion zur Kommando verarbeitung und ausführung
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
        elif "fahr im kreis" in text:
            self.twist.linear.x = 0.3
            self.twist.angular.z = -0.6 
        elif "halt" in text:
            # Anhalten
            self.get_logger().info(f"Anhalten")
            pass
        else:
            # Unbekannter Befehl, nicht reagieren
            return


        self.pub.publish(self.twist)
# Terminal Ausgabe der Bewegungsparameter
        # self.get_logger().info(f"Bewegung gesendet: linear.x={self.twist.linear.x}, angular.z={self.twist.angular.z}")


#----------------Main Programm----------------
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
