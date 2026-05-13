import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardNode(Node):
    def __init__(self):
        super().__init__('keyboard_node')
        
        # Creamos un publisher al tópico 'cmd'
        self.publisher_ = self.create_publisher(String, 'cmd', 10)
        
        self.get_logger().info("Control listo: usa W A S D")

    def get_key(self):
        """
        Lee una tecla del teclado sin necesidad de presionar ENTER
        """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """
        Loop principal que detecta teclas y publica comandos
        """
        while True:
            key = self.get_key()

            msg = String()

            if key == 'w':
                msg.data = "UP"
            elif key == 's':
                msg.data = "DOWN"
            elif key == 'a':
                msg.data = "LEFT"
            elif key == 'd':
                msg.data = "RIGHT"
            elif key == 'q':
                self.get_logger().info("Saliendo...")
                break
            else:
                continue

            # Publicar mensaje
            self.publisher_.publish(msg)

            # Mostrar en consola
            self.get_logger().info(f"Enviado: {msg.data}")


def main():
    rclpy.init()
    node = KeyboardNode()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
