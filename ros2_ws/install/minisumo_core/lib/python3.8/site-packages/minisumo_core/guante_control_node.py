import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import math

class KalmanFilter:
    def __init__(self, q=0.001, r=0.01):
        self.q = q 
        self.r = r 
        self.p = 1.0 
        self.x = 0.0 

    def update(self, measurement):
        self.p = self.p + self.q
        k = self.p / (self.p + self.r)
        self.x = self.x + k * (measurement - self.x)
        self.p = (1 - k) * self.p
        return self.x

class GuanteControlNode(Node):
    def __init__(self):
        super().__init__('guante_control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- CONFIGURACIÓN SERIAL ---
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            self.get_logger().info("Conectado al Guante en /dev/ttyUSB0")
        except Exception as e:
            self.get_logger().error(f"Error Serial: {e}")

        # --- FILTROS DE KALMAN PARA CADA EJE CLAVE ---
        self.k_acc_z = KalmanFilter()
        self.k_gyr_x = KalmanFilter()
        self.k_acc_x = KalmanFilter()
        self.k_gyr_z = KalmanFilter()

        # Parámetros de velocidad (puedes ajustarlos aquí)
        self.base_linear = 0.7
        self.base_angular = 1.0
        self.deadzone = 0.50 # Umbral para ignorar movimientos pequeños

        self.timer = self.create_timer(0.002, self.procesar_datos)

    def procesar_datos(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                parts = line.split(',')
                data = {}
                for part in parts:
                    if ':' in part:
                        key, val = part.split(':')
                        data[key] = float(val)

                # 1. Obtener y filtrar datos necesarios
                # Adelante/Atrás: Acc Z e inclinación en X
                acc_z = self.k_acc_z.update(data.get('Acc_Z_Adelante', 0.0))
                gyr_x = self.k_gyr_x.update(data.get('Gyr_X_RotLat', 0.0))
                
                # Giros: Acc X y rotación en Z
                acc_x = self.k_acc_x.update(data.get('Acc_X_Lateral', 0.0))
                gyr_z = self.k_gyr_z.update(data.get('Gyr_Z_RotAdel', 0.0))

                msg = Twist()

                # --- LÓGICA LINEAL (EJE X del Robot) ---
                # Combinamos la gravedad en Z con la velocidad de rotación en X
                val_lineal = acc_z*0.01 + (gyr_x*0.1) 
                
                if abs(val_lineal) > self.deadzone:
                    signo = 1.0 if val_lineal > 0 else -1.0
                    # Velocidad = Base (0.7) + Proporcional al movimiento
                    msg.linear.x = signo * (self.base_linear + abs(val_lineal) * 0.5)
                else:
                    msg.linear.x = 0.0

                # --- LÓGICA ANGULAR (EJE Z del Robot) ---
                # Combinamos inclinación lateral (Acc X) con rotación de muñeca (Gyr Z)
                val_angular = acc_x*0.01 + (gyr_z*0.1)

                if abs(val_angular) > self.deadzone:
                    signo = -1.0 if val_angular > 0 else 1.0 # Invertir si es necesario según tu comodidad
                    # Velocidad = Base (2.0) + Proporcional
                    msg.angular.z = signo * (self.base_angular + abs(val_angular) * 1)
                else:
                    msg.angular.z = 0.0

                self.publisher_.publish(msg)

            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = GuanteControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()