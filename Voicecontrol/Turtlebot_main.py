import rclpy                                #type: ignore
from voicecontrol import VoiceControlNode   #type: ignore

def main(args=None):
    rclpy.init(args=args)
    voice_node = VoiceControlNode()
    try:
        rclpy.spin(voice_node)
    except KeyboardInterrupt:
        pass
    finally:
        voice_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
