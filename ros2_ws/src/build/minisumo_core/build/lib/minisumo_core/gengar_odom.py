import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import qos_profile_sensor_data
import math
import time

class GengarOdometry(Node):
    def __init__(self):
        super().__init__('gengar_odometry_node')
        
        # Suscriptores (Escuchamos al ESP32)
        self.sub_imu = self.create_subscription(Imu, '/imu/data_filtered', self.imu_cb, qos_profile_sensor_data)
        self.sub_enc = self.create_subscription(Twist, '/motor/telemetry', self.enc_cb, qos_profile_sensor_data)
        
        # Publicador de Transformaciones (Le hablamos a RViz)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Variables de estado
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.v_encoder = 0.0
        self.v_fused = 0.0
        self.last_time = time.time()
        
        self.get_logger().info("¡Cerebro de Odometría de Gengar Iniciado!")

    def enc_cb(self, msg):
        # Asumimos que mandas la velocidad de la rueda en el eje X de Twist
        self.v_encoder = msg.linear.x

    def imu_cb(self, msg):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # 1. Obtenemos datos del IMU
        ax = msg.linear_acceleration.x  # Aceleración hacia adelante
        omega = msg.angular_velocity.z  # Giroscopio (Giro sobre su propio eje)
        
        # 2. FUSIÓN DE SENSORES (Filtro Complementario) para la velocidad
        alpha = 0.98
        self.v_fused = alpha * (self.v_fused + ax * dt) + (1.0 - alpha) * self.v_encoder
        
        # 3. Calculamos la nueva posición (Cinemática)
        self.theta += omega * dt
        self.x += self.v_fused * math.cos(self.theta) * dt
        self.y += self.v_fused * math.sin(self.theta) * dt
        
        # 4. Publicamos a RViz (TF)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'        # El mapa mundial
        t.child_frame_id = 'base_link'    # Tu robot
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Convertimos ángulo Theta a Cuaternión para RViz
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    nodo = GengarOdometry()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
