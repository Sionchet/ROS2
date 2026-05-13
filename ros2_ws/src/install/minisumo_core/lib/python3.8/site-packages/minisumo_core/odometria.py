import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, TransformStamped
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
import math
import time

class CalculadoraOdometria(Node):
    def __init__(self):
        super().__init__('odometria_node')
        
        # Suscripciones con QoS para micro-ROS
        self.sub_motor = self.create_subscription(Twist, '/motor/telemetry', self.motor_callback, qos_profile_sensor_data)
        self.sub_imu = self.create_subscription(Imu, '/imu/data_filtered', self.imu_callback, qos_profile_sensor_data)
        
        # Publicadores
        self.pub_odom = self.create_publisher(Pose2D, '/posicion_estimada', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Estado del robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        
        self.ultimo_tiempo = self.get_clock().now()
        self.timer = self.create_timer(0.05, self.actualizar_posicion)
        self.get_logger().info('Cerebro de Odometría de Gengar listo.')

    def motor_callback(self, msg):
        self.v = msg.linear.y 

    def imu_callback(self, msg):
        self.w = msg.angular_velocity.z

    def actualizar_posicion(self):
        tiempo_actual = self.get_clock().now()
        # Calculamos dt usando el reloj de ROS2
        dt = (tiempo_actual - self.ultimo_tiempo).nanoseconds / 1e9
        self.ultimo_tiempo = tiempo_actual
        
        # VALIDACIÓN: Solo calculamos si ha pasado tiempo real
        if dt > 0.0001:
            # 1. Integración de posición
            self.theta += self.w * dt
            self.x += self.v * math.cos(self.theta) * dt
            self.y += self.v * math.sin(self.theta) * dt
            
            # 2. Publicar Pose2D para Totoro.csv
            msg_pose = Pose2D()
            msg_pose.x = self.x
            msg_pose.y = self.y
            msg_pose.theta = self.theta
            self.pub_odom.publish(msg_pose)
            
            # 3. Publicar TF para RViz (Para que el modelo 3D se mueva)
            t = TransformStamped()
            t.header.stamp = tiempo_actual.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_footprint'
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            
            # Convertir ángulo a Quaternion (formato que entiende ROS2)
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = math.sin(self.theta / 2.0)
            t.transform.rotation.w = math.cos(self.theta / 2.0)
            
            self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    nodo = CalculadoraOdometria()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()