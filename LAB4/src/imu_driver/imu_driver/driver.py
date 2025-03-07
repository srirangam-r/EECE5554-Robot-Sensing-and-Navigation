import rclpy
from rclpy.node import Node
import serial
import math

from custom_interfaces.msg import IMUmsg   
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Header
from builtin_interfaces.msg import Time   

def euler_to_quaternion(yaw, pitch, roll):
    yaw_rad = math.radians(yaw)
    pitch_rad = math.radians(pitch)
    roll_rad = math.radians(roll)

    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)

    q_w = cr * cp * cy + sr * sp * sy
    q_x = sr * cp * cy - cr * sp * sy
    q_y = cr * sp * cy + sr * cp * sy
    q_z = cr * cp * sy - sr * sp * cy

    return (q_w, q_x, q_y, q_z)

class IMUPub(Node):

    def __init__(self):
        super().__init__('imu_driver')
        self.publisher_ = self.create_publisher(IMUmsg, 'imu', 10)
        self.declare_parameter('port', '/dev/ttyUSB0')
        timer_period = 0.01
        serial_port = self.get_parameter('port').value
        self.timer = self.create_timer(timer_period, self.callback)
        self.port = serial.Serial(serial_port, 115200, timeout=3.0)
        command = '$VNWRG,07,40*59'+'\r\n'
        self.port.write(command.encode())
        self.get_logger().info('Initialising IMU...')    

    def callback(self):
        msg = IMUmsg()
        header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "IMU1_Frame"

        line = str(self.port.readline().decode('ascii', errors='replace'))
        # print(len(line.split('*')[0].split(',')[1:]))
        values = list(map(float, line.split('*')[0].split(',')[1:]))
        if len(values) == 12:
            yaw, pitch, roll, magx, magy, magz, accelx, accely, accelz, gyrox, gyroy, gyroz = values[:12]
            quaternion = euler_to_quaternion(yaw, pitch, roll)
            msg.imu.orientation.w, msg.imu.orientation.x, msg.imu.orientation.y, msg.imu.orientation.z  = quaternion
            msg.imu.angular_velocity.x, msg.imu.angular_velocity.y, msg.imu.angular_velocity.z = gyrox,gyroy,gyroz
            msg.imu.linear_acceleration.x, msg.imu.linear_acceleration.y, msg.imu.linear_acceleration.z = accelx, accely, accelz
            msg.mag_field.magnetic_field.x, msg.mag_field.magnetic_field.y, msg.mag_field.magnetic_field.z = magx/10000, magy/10000, magz/10000
            msg.raw_data = line
            self.publisher_.publish(msg)
            self.get_logger().info("Published....")    
            


def main(args=None):
    rclpy.init(args=args)
    imu_driver = IMUPub()
    rclpy.spin(imu_driver)
    imu_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()