import rclpy
from rclpy.node import Node

from queue import Queue
from threading import Event

from sensor_msgs.msg import NavSatFix


from .ublox_gnss import UbloxGnss

class UbloxGnssNode(Node):
    def __init__(self):
        super().__init__('ublox_gnss_node')
        self.get_logger().info('Ublox GNSS Node has been started.')
        
        # declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'gps')
        
        # get parameters
        self.port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.get_logger().info(f'Using serial port: {self.port}')
        self.get_logger().info(f'Using baudrate: {self.baudrate}')
        
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        
        self.fix_pub_timer = self.create_timer(0.1, self.on_fix_pub_timer)

        self.send_queue = Queue()
        self.receive_queue = Queue()
        self.stop_event = Event()
    
        with UbloxGnss(
            self.port,
            int(self.baudrate),
            float(3),
            self.stop_event,
            sendqueue=self.send_queue,
            receivequeue=self.receive_queue,
            idonly=False,
            enableubx=True,
            enablenmea=False,
            showhacc=True,
            verbose=True,
        ) as gna:
            gna.run()
            
    def on_fix_pub_timer(self):
        try:
            parsed_data = self.receive_queue.get()
            if hasattr(parsed_data, "identity"):
                idy = parsed_data.identity
                if idy == 'NAV-PVT':
                    navsat_fix_msg = NavSatFix()
                    navsat_fix_msg.header.stamp = self.get_clock().now().to_msg()
                    navsat_fix_msg.header.frame_id = self.frame_id
                    navsat_fix_msg.latitude = parsed_data.lat
                    navsat_fix_msg.longitude = parsed_data.lon
                    navsat_fix_msg.altitude = parsed_data.height / 1000.0 # mm to m

                    #TODO: add covariance matrix

                    self.fix_pub.publish(navsat_fix_msg)  
        
        except Exception as e:
            self.get_logger().error(f"Error: {e}")
    
    
def main(args=None):
    rclpy.init(args=args)
    node = UbloxGnssNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()