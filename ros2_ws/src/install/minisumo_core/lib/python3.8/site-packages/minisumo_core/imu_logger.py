import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import csv
import os
from datetime import datetime

class ImuCsvLogger(Node):
    def __init__(self):
        super().__init__('imu_csv_logger')
        
        # Crear el archivo CSV con la fecha actual
        filename = f"imu_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.filepath = os.path.join(os.getcwd(), filename)
        
        # Preparar el archivo y escribir los encabezados
        with open(self.filepath, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'accel_x', 'accel_y', 'accel_z', 'gyro_x', 'gyro_y', 'gyro_z'])
            
        self.get_logger().info(f'Guardando datos de la IMU en: {self.filepath}')

        # Suscribirse al tópico que viene directo del ESP32
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        # Extraer los datos (que tu ESP32 ya mandó filtrados)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        timestamp = self.get_clock().now().nanoseconds / 1e9

        # Escribir la nueva fila en el CSV
        with open(self.filepath, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([timestamp, ax, ay, az, gx, gy, gz])

def main(args=None):
    rclpy.init(args=args)
    node = ImuCsvLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Grabación de CSV detenida.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
