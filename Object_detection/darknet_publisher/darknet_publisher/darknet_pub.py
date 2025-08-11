#!/usr/bin/env python3
import os
import json
import time
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# Darknet-Pfad anpassen, falls nötig
import sys
sys.path.append("/home/basti/darknet")
import darknet

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class YoloPublisher(Node):
    def __init__(self):
        super().__init__('yolo_object_publisher')

        # Parameter: image topic (standard /image_raw), threshold
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('thresh', 0.5)
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.thresh = float(self.get_parameter('thresh').get_parameter_value().double_value)

        self.publisher_ = self.create_publisher(String, 'yolo_objects', 10)

        # --- Darknet-Dateipfade (anpassen falls nötig) ---
        cfg = os.path.expanduser('~/darknet/data/yolo-aiv2/yolov4-tiny-custom.cfg')
        data = os.path.expanduser('~/darknet/data/yolo-aiv2/obj.data')
        weights = os.path.expanduser('~/darknet/data/yolo-aiv2/yolov4-tiny-custom_best.weights')

        for p in (cfg, data, weights):
            if not os.path.exists(p):
                self.get_logger().error(f"Pfad existiert nicht: {p}")
                raise FileNotFoundError(p)

        # --- Darknet Netzwerk laden (Python API) ---
        self.network, self.class_names, self.class_colors = darknet.load_network(
            cfg,
            data,
            weights,
            batch_size=1
        )
        self.net_w = darknet.network_width(self.network)
        self.net_h = darknet.network_height(self.network)
        self.get_logger().info(f"Darknet geladen (w={self.net_w}, h={self.net_h})")

        # Lock für Darknet-Aufrufe (Thread-Safety)
        self.darknet_lock = threading.Lock()

        # --- Subscriber für image topic ---
        qos = QoSProfile(depth=5)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        qos.history = QoSHistoryPolicy.KEEP_LAST
        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_stamp = (0, 0)  # (sec, nanosec)
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()

        self.create_subscription(Image, image_topic, self.image_callback, qos)
        self.get_logger().info(f"Subscribed to image topic: {image_topic}")

        # --- Worker-Thread für Detection ---
        self._stop_worker = False
        self.worker_thread = threading.Thread(target=self._detection_loop, daemon=True)
        self.worker_thread.start()

    def image_callback(self, msg: Image):
        """Speichert nur das NEUESTE Frame (basierend auf header stamp) und signalisiert Worker."""
        try:
            # konvertiere (BGR8) — cv_bridge liefert ein numpy BGR-Array
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge Fehler: {e}")
            return

        # Verwende header stamp um sicher zu sein, dass wir nur neuere Frames verwenden
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self.frame_lock:
            if stamp > self.latest_stamp:
                self.latest_frame = cv_image.copy()
                self.latest_stamp = stamp
                # signal an worker: neues Frame verfügbar
                self.frame_event.set()

    def _detection_loop(self):
        """Wartet auf neue Frames und verarbeitet jeweils das aktuellste Frame genau einmal."""
        while rclpy.ok() and not self._stop_worker:
            # Warte auf Event (Zeitlimit, damit wir shutdown-checks machen können)
            got = self.frame_event.wait(timeout=1.0)
            if not got:
                continue
            # Hol das aktuellste Frame (atomar)
            with self.frame_lock:
                frame = None
                if self.latest_frame is not None:
                    frame = self.latest_frame
                    # Wir setzen latest_frame zurück, damit es nicht wiederverwendet wird
                    self.latest_frame = None
                    self.latest_stamp = (0, 0)
                # Reset event (falls noch keine neuen Frames)
                self.frame_event.clear()

            if frame is None:
                continue

            # Run detection (schütze Darknet-Aufrufe mit Lock)
            try:
                self._run_yolo_on_frame(frame)
            except Exception as e:
                self.get_logger().error(f"Detection-Fehler: {e}")

    """
    def _run_yolo_on_frame(self, frame):
        # resize, BGR->RGB, in Darknet-Image kopieren
        frame_resized = cv2.resize(frame, (self.net_w, self.net_h), interpolation=cv2.INTER_LINEAR)
        frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)

        # Darknet calls guarded by lock
        with self.darknet_lock:
            darknet_img = darknet.make_image(self.net_w, self.net_h, 3)
            darknet.copy_image_from_bytes(darknet_img, frame_rgb.tobytes())
            detections = darknet.detect_image(self.network, self.class_names, darknet_img, thresh=self.thresh)
            darknet.free_image(darknet_img)

        # detections -> liste von dicts
        results = []
        
        for label, confidence, bbox in detections:
            x, y, w, h = bbox
            results.append({
                'label': label,
                'confidence': float(confidence),
                'bbox': {'x': int(x), 'y': int(y), 'w': int(w), 'h': int(h)}
            })
        """
    
    def _run_yolo_on_frame(self, frame):
        # resize, BGR->RGB, in Darknet-Image kopieren
        frame_resized = cv2.resize(frame, (self.net_w, self.net_h), interpolation=cv2.INTER_LINEAR)
        frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)

        # Darknet calls guarded by lock
        with self.darknet_lock:
            darknet_img = darknet.make_image(self.net_w, self.net_h, 3)
            darknet.copy_image_from_bytes(darknet_img, frame_rgb.tobytes())
            detections = darknet.detect_image(self.network, self.class_names, darknet_img, thresh=self.thresh)
            darknet.free_image(darknet_img)

        # Für jede Erkennung: "label: 78%"
        for det in detections:
            # det kann (label, confidence, bbox)
            try:
                label = det[0]
                conf_raw = det[1]
            except Exception:
                # defensiv: falls Format anders ist, skip
                self.get_logger().warning(f"Unbekanntes detection-format: {det}")
                continue

            # Konvertiere confidence robust zu float
            try:
                conf = float(conf_raw)
            except Exception:
                # z.B. "78%" oder bytes -> entferne '%' und versuche erneut
                s = str(conf_raw).strip()
                if s.endswith('%'):
                    s = s[:-1]
                try:
                    conf = float(s)
                except Exception:
                    # absolut defensiv: wenn nichts geht, überspringen
                    self.get_logger().warning(f"Konnte confidence nicht parsen: {conf_raw}")
                    continue

            # Normalisiere: falls conf im 0..1 Bereich, in Prozent umwandeln
            if conf <= 1.5:
                pct = int(round(conf * 100))
            else:
                pct = int(round(conf))

            # Clamp 0..100
            pct = max(0, min(100, pct))

            text = f"{label} {pct}%"
            ros_msg = String()
            ros_msg.data = text
            self.publisher_.publish(ros_msg)
            self.get_logger().info(f"Published: {text}")


    def destroy(self):
        # sauber stoppen
        self._stop_worker = True
        # signalisiere Worker (falls er wartet)
        self.frame_event.set()
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=2.0)
        # dann Node zerstören
        try:
            self.destroy_node()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = YoloPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node beendet (KeyboardInterrupt)')
    finally:
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
