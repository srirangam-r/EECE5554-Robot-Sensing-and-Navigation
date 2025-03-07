import rclpy
from rclpy.node import Node
import serial
import utm

from custom_interfaces.msg import GPSmsg   
from std_msgs.msg import Header
from builtin_interfaces.msg import Time                      


class GPSPub(Node):

    def __init__(self):
        super().__init__('gps_driver')
        self.publisher_ = self.create_publisher(GPSmsg, 'gps', 10)
        self.declare_parameter('port', '/dev/ttyACM0')
        timer_period = 0.05
        serial_port = self.get_parameter('port').value
        self.timer = self.create_timer(timer_period, self.callback)
        self.port = serial.Serial(serial_port, 4800, timeout=3.0)
        self.get_logger().info('Initialising GPS...')    

    def callback(self):
        msg = GPSmsg()
        header = Header()
        time = Time()

        line = self.port.readline().decode().strip()
        if line.startswith("$GPGGA"):
            gps_data = line.split(',') 
            if gps_data[1] and gps_data[2] and gps_data[4] and gps_data[9]:
                lat_degrees = int(gps_data[2][:2])
                lat_minutes = float(gps_data[2][2:])
                lat = lat_degrees + lat_minutes/60
                if gps_data[3] == "S":
                    lat = -lat

                lon_degrees = int(gps_data[4][:3])
                lon_minutes = float(gps_data[4][3:])
                lon = lon_degrees + lon_minutes/60
                if gps_data[5] == "W":
                    lon = -lon      

                utm_data = utm.from_latlon(lat,lon)
                
                # gpgga_time = int(gps_data[1][:2])*3600 + int(gps_data[1][2:4])*60 + float(gps_data[1][4:]) 
                # time.sec = int(gpgga_time)
                # time.nanosec = int((gpgga_time - int(gpgga_time))*1e9)

                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = "GPS1_Frame"
                msg.header = header

                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = float(gps_data[9])
                msg.utm_easting = utm_data[0]
                msg.utm_northing = utm_data[1]
                msg.zone = utm_data[2]
                msg.letter = utm_data[3]

                self.publisher_.publish(msg)
                self.get_logger().info("Published....")    


def main(args=None):
    rclpy.init(args=args)
    gps_driver = GPSPub()
    rclpy.spin(gps_driver)
    gps_driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()