import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist, Pose2D
from rclpy.qos import qos_profile_sensor_data 
import csv
import os
import time

class NodoLoggerCSV(Node):
    def __init__(self):
        super().__init__('logger_csv')
        
        self.ruta_archivo = os.path.expanduser('~/ros2_ws/Totoro.csv')
        self.archivo = open(self.ruta_archivo, 'w', newline='')
        self.escritor = csv.writer(self.archivo)
        
        self.escritor.writerow([
            'Tiempo(s)', 
            'Ax_Cruda', 'Ay_Cruda', 'Gz_Cruda', 
            'Ax_Filt', 'Ay_Filt', 'Gz_Filt', 
            'Enc_Crudo', 'Enc_Filt', 'Setpoint_Motor',
            'X_Est', 'Y_Est', 'Theta_Est'
        ])
        
        self.datos = {
            'ax_raw': 0.0, 'ay_raw': 0.0, 'gz_raw': 0.0,
            'ax_filt': 0.0, 'ay_filt': 0.0, 'gz_filt': 0.0,
            'enc_raw': 0.0, 'enc_filt': 0.0, 'setpoint': 0.0,
            'x': 0.0, 'y': 0.0, 'theta': 0.0
        }
        
        
        self.sub_raw = self.create_subscription(Imu, '/imu/data_raw', self.cb_raw, qos_profile_sensor_data)
        self.sub_filt = self.create_subscription(Imu, '/imu/data_filtered', self.cb_filt, qos_profile_sensor_data)
        self.sub_motor = self.create_subscription(Twist, '/motor/telemetry', self.cb_motor, qos_profile_sensor_data)
        
        
        self.sub_odom = self.create_subscription(Pose2D, '/posicion_estimada', self.cb_odom, 10)
        
        self.inicio_tiempo = time.time()
        self.timer = self.create_timer(0.1, self.guardar_fila_csv)
        
        self.get_logger().info(f'Grabando en: {self.ruta_archivo}')

    def cb_raw(self, msg):
        self.datos['ax_raw'] = msg.linear_acceleration.x
        self.datos['ay_raw'] = msg.linear_acceleration.y
        self.datos['gz_raw'] = msg.angular_velocity.z

    def cb_filt(self, msg):
        self.datos['ax_filt'] = msg.linear_acceleration.x
        self.datos['ay_filt'] = msg.linear_acceleration.y
        self.datos['gz_filt'] = msg.angular_velocity.z

    def cb_motor(self, msg):
        self.datos['enc_raw'] = msg.linear.x
        self.datos['enc_filt'] = msg.linear.y
        self.datos['setpoint'] = msg.linear.z

    def cb_odom(self, msg):
        self.datos['x'] = msg.x
        self.datos['y'] = msg.y
        self.datos['theta'] = msg.theta

    def guardar_fila_csv(self):
        t_actual = time.time() - self.inicio_tiempo
        self.escritor.writerow([
            round(t_actual, 3),
            round(self.datos['ax_raw'], 4), round(self.datos['ay_raw'], 4), round(self.datos['gz_raw'], 4),
            round(self.datos['ax_filt'], 4), round(self.datos['ay_filt'], 4), round(self.datos['gz_filt'], 4),
            round(self.datos['enc_raw'], 4), round(self.datos['enc_filt'], 4), round(self.datos['setpoint'], 4),
            round(self.datos['x'], 4), round(self.datos['y'], 4), round(self.datos['theta'], 4)
        ])
        self.archivo.flush()

def main(args=None):
    rclpy.init(args=args)
    nodo = NodoLoggerCSV()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.archivo.close()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
