import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

class KalmanFilter:
    def __init__(self, q=0.01, r=0.1):
        self.q = q # Ruido de proceso
        self.r = r # Ruido de medición
        self.p = 1.0 # Error de estimación
        self.x = 0.0 # Valor estimado (estado)
        self.k = 0.0 # Ganancia de Kalman

    def update(self, measurement):
        # Predicción
        self.p = self.p + self.q
        # Actualización
        self.k = self.p / (self.p + self.r)
        self.x = self.x + self.k * (measurement - self.x)
        self.p = (1 - self.k) * self.p
        return self.x

class GuanteControlNode(Node):
    def __init__(self):
        super().__init__('guante_control_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # --- CONFIGURACIÓN SERIAL ---
        # Ajusta '/dev/ttyUSB0' según tu puerto (en Windows suele ser 'COMx')
        try:
            self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.1)
            self.get_logger().info("Puerto Serial conectado exitosamente.")
        except Exception as e:
            self.get_logger().error(f"No se pudo abrir el puerto serial: {e}")

        # --- FILTROS DE KALMAN ---
        self.kalman_linear = KalmanFilter(q=0.001, r=0.05)
        self.kalman_angular = KalmanFilter(q=0.001, r=0.05)

        # Timer para procesar datos a 50Hz
        self.timer = self.create_timer(0.02, self.procesar_datos)

    def procesar_datos(self):
        if self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8').strip()
                # El formato esperado es: Acc_Z_Adelante:0.12,Acc_Y_Arriba:9.8,Acc_X_Lateral:0.05...
                parts = line.split(',')
                
                # Extraer valores usando los nombres que pusimos en el ESP32
                data = {}
                for part in parts:
                    if ':' in part:
                        key, val = part.split(':')
                        data[key] = float(val)

                # --- LÓGICA DE CONTROL ---
                # Eje Z (Adelante/Atrás) -> Velocidad Lineal
                # Eje X (Lateral) -> Velocidad Angular
                
                raw_z = data.get('Acc_Z_Adelante', 0.0)
                raw_x = data.get('Acc_X_Lateral', 0.0)

                # Aplicar Kalman
                clean_z = self.kalman_linear.update(raw_z)
                clean_x = self.kalman_angular.update(raw_x)

                msg = Twist()

                # ZONA MUERTA (Deadzone) para evitar que el robot se mueva solo
                # Ajusta estos valores (0.15) según la sensibilidad deseada
                if abs(clean_z) > 0.15:
                    # Mapeo: Si Z es positivo (mano adelante), lineal.x es positivo
                    msg.linear.x = clean_z * 0.5 # Factor de escala 0.5
                else:
                    msg.linear.x = 0.0

                if abs(clean_x) > 0.15:
                    # Mapeo: Inclinación lateral -> Giro
                    msg.angular.z = -clean_x * 1.5 # El signo negativo depende de la dirección deseada
                else:
                    msg.angular.z = 0.0

                self.publisher_.publish(msg)

            except Exception as e:
                # Silenciamos errores de lectura parcial de línea
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

if __name__ == '__main__':
    main()
