import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        self.ser = serial.Serial('/dev/ttyUSB0', 115200)

        self.subscription = self.create_subscription(
            String,
            'cmd',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        comando = msg.data + "\n"
        self.ser.write(comando.encode())
        self.get_logger().info(f"Enviado a ESP32: {msg.data}")

def main():
    rclpy.init()
    node = SerialNode()
    rclpy.spin(node)
