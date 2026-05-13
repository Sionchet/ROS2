import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from pynput import keyboard
import time
import math

class TeleopGengar(Node):
    def __init__(self):
        super().__init__('teleop_gengar')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Velocidades base
        self.vel_lineal = 0.2 
        self.vel_angular = 1.0
        
        self.ejecutando_macro = False
        
        self.get_logger().info('--- CONTROL DE GENGAR INICIADO ---')
        self.get_logger().info('W/S: Adelante/Atrás | A/D: Girar')
        self.get_logger().info('R/F: Aumentar/Reducir Velocidad')
        self.get_logger().info('O: Giro 360 | I: Macro Der | U: Macro Izq | Y: Macro Reversa')
        self.get_logger().info('Q: Salir')
        self.listener = keyboard.Listener(on_press=self.al_presionar, on_release=self.al_soltar)
        self.listener.start()

    def enviar_comando(self, lx, az):
        msg = Twist()
        msg.linear.x = float(lx)
        msg.angular.z = float(az)
        self.publisher.publish(msg)

    def rutina_compleja(self, tipo):
        self.ejecutando_macro = True
        
        if tipo == 'O':
            self.get_logger().info('Macro O: Giro de 360 grados sobre su eje')
            self.enviar_comando(0.0, 1.0)
            time.sleep(1.8)
            
        elif tipo == 'I':
            self.get_logger().info('Macro I: Avanza 50cm y gira a la derecha 30cm')
            self.enviar_comando(self.vel_lineal, 0.0)
            time.sleep(2.5)
            self.enviar_comando(0.0, 0.0)
            time.sleep(0.5)
            self.enviar_comando(0.0, -1.0)
            time.sleep(0.7)
            self.enviar_comando(self.vel_lineal, 0.0)
            time.sleep(1.5)
            
        elif tipo == 'U':
            self.get_logger().info('Macro U: Avanza 50cm y gira a la izquierda 30cm')
            self.enviar_comando(self.vel_lineal, 0.0)
            time.sleep(2.5)
            self.enviar_comando(0.0, 0.0)
            time.sleep(0.5)
            self.enviar_comando(0.0, 1.0)
            time.sleep(0.7)
            self.enviar_comando(self.vel_lineal, 0.0)
            time.sleep(1.5)
            
        elif tipo == 'Y':
            self.get_logger().info('Macro Y: Reversa, media vuelta y seguir de frente')
            self.enviar_comando(-self.vel_lineal, 0.0)
            time.sleep(1.0)
            self.enviar_comando(0.0, 1.0)
            time.sleep(0.9)
            self.enviar_comando(self.vel_lineal, 0.0)
            time.sleep(2.0)

        self.enviar_comando(0.0, 0.0)
        self.ejecutando_macro = False
        self.get_logger().info('Macro terminada.')

    def al_presionar(self, key):
        if self.ejecutando_macro:
            return 

        try:
            k = key.char.lower()
            if k == 'w':
                self.enviar_comando(self.vel_lineal, 0.0)
            elif k == 's':
                self.enviar_comando(-self.vel_lineal, 0.0)
            elif k == 'a':
                self.enviar_comando(0.0, self.vel_angular)
            elif k == 'd':
                self.enviar_comando(0.0, -self.vel_angular)
            elif k == 'r':
                self.vel_lineal += 0.05
                self.get_logger().info(f'Velocidad Lineal Aumentada: {self.vel_lineal:.2f} m/s')
            elif k == 'f':
                self.vel_lineal = max(0.05, self.vel_lineal - 0.05) # Evita velocidad negativa
                self.get_logger().info(f'Velocidad Lineal Reducida: {self.vel_lineal:.2f} m/s')
            elif k == 'o':
                self.rutina_compleja('O')
            elif k == 'i':
                self.rutina_compleja('I')
            elif k == 'u':
                self.rutina_compleja('U')
            elif k == 'y':
                self.rutina_compleja('Y')
        except AttributeError:
            pass # Teclas especiales (Shift, Ctrl) se ignoran

    def al_soltar(self, key):
        if self.ejecutando_macro:
            return

        try:
            k = key.char.lower()
            # Si suelta una tecla de movimiento, el robot frena
            if k in ['w', 'a', 's', 'd']:
                self.enviar_comando(0.0, 0.0)
            elif k == 'q':
                self.get_logger().info('Saliendo del control...')
                return False # Esto detiene el listener del teclado
        except AttributeError:
            pass

def main():
    rclpy.init()
    node = TeleopGengar()
    
    try:
        # Usamos spin_once en un bucle para poder salir limpiamente cuando el listener muera
        while rclpy.ok() and node.listener.is_alive():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.enviar_comando(0.0, 0.0) # Frenar al cerrar
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()