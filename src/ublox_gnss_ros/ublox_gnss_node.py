import rclpy
from rclpy.node import Node

from queue import Queue
from threading import Event

from sensor_msgs.msg import NavSatFix, NavSatStatus
from nmea_msgs.msg import Sentence
from rtcm_msgs.msg import Message as RtcmMessage


from .ublox_gnss import UbloxGnss

class UbloxGnssNode(Node):
    def __init__(self):
        super().__init__('ublox_gnss_node')
        self.get_logger().info('Ublox GNSS Node has been started.')
        
        # declare parameters
        self.declare_parameter('serial.port', '/dev/ttyACM0')
        self.declare_parameter('serial.baudrate', 115200)
        self.declare_parameter('serial.timeout', 3.0)
        self.declare_parameter('meas_rate', 1000)
        self.declare_parameter('nav_rate', 1)
        self.declare_parameter('nav_prio_rate', 1)
        self.declare_parameter('frame_id', 'gps')
        self.declare_parameter('publish.nmea', True)
        # self.declare_parameter('publish.imu', True)
        
        self.declare_parameter('verbose', False)
        
        # get parameters
        self.serial_port = self.get_parameter('serial.port').get_parameter_value().string_value
        self.serial_baudrate = self.get_parameter('serial.baudrate').get_parameter_value().integer_value
        self.serial_timeout = self.get_parameter('serial.timeout').get_parameter_value().double_value
        
        self.gnss_frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        
        self.meas_rate = self.get_parameter('meas_rate').get_parameter_value().integer_value
        self.nav_rate = self.get_parameter('nav_rate').get_parameter_value().integer_value
        self.nav_prio_rate = self.get_parameter('nav_prio_rate').get_parameter_value().integer_value
        
        self.publish_nmea = self.get_parameter('publish.nmea').get_parameter_value().bool_value
        # self.publish_imu = self.get_parameter('publish.imu').get_parameter_value().bool_value
        
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
        
        # print parameters
        self.get_logger().info(f'Using serial port: {self.serial_port}')
        self.get_logger().info(f'Using serial baudrate: {self.serial_baudrate}')
        self.get_logger().info(f'Using serial timeout: {self.serial_timeout}')
        self.get_logger().info(f'Using frame id: {self.gnss_frame_id}')
        self.get_logger().info(f'Using meas rate: {self.meas_rate}')
        self.get_logger().info(f'Using nav rate: {self.nav_rate}')
        self.get_logger().info(f'Using nav prio rate: {self.nav_prio_rate}')
        self.get_logger().info(f'Using publish nmea: {self.publish_nmea}')
        # self.get_logger().info(f'Using publish imu: {self.publish_imu}')
        self.get_logger().info(f'Using verbose: {self.verbose}')
        
        # init topics
        self.fix_pub = self.create_publisher(
            NavSatFix, '/fix', 10)
        
        if self.publish_nmea:
            self.nmea_pub = self.create_publisher(
                Sentence, '/nmea', 10)
            
        self.rtcm_sub = self.create_subscription(
            RtcmMessage,
            '/rtcm',
            self.on_rtcm_msg_rx_callback,
            10
        )
        
        self.fix_pub_timer = self.create_timer(
            0.01, self.on_fix_pub_timer)

        # init ublox gnss
        self.send_queue = Queue()
        self.receive_queue = Queue()
        self.stop_event = Event()
    
        self.ublox_gnss = UbloxGnss(
            self.serial_port,
            int(self.serial_baudrate),
            float(self.serial_timeout),
            self.stop_event,
            idonly=False,
            enableubx=True,
            enablenmea=True,
            showhacc=True,
            verbose=False,
            measrate=self.meas_rate,
            navrate=self.nav_rate,
            navpriorate=self.nav_prio_rate,
        )
        
        self.ublox_gnss.run()
        
        self.msg_cnt = 0
        self.last_msg_time = None
        
        self.last_nav_cov = None
        self.last_nav_data = None
    
    def on_rtcm_msg_rx_callback(self, msg):
        # put to send queue for sending to ublox gnss module
        if self.ublox_gnss is not None:
            self.ublox_gnss.send_rtcm(msg.message)
            
    def on_fix_pub_timer(self):
        try:
            
            nav_cov = self.ublox_gnss.get_nav_cov()
            if nav_cov is not None:
                self.last_nav_cov = nav_cov
                
            nav_data = self.ublox_gnss.get_nav_data()
            if nav_data is not None:
                lat, lon, alt, carr_soln, fix_type, gnss_fix_ok = nav_data
                navsatfix_msg = NavSatFix()
                navsatfix_msg.header.stamp = self.get_clock().now().to_msg()
                navsatfix_msg.header.frame_id = self.gnss_frame_id
                navsatfix_msg.latitude = lat
                navsatfix_msg.longitude = lon
                navsatfix_msg.altitude = alt
                if self.last_nav_cov is not None:
                    navsatfix_msg.position_covariance = self.last_nav_cov
                navsatfix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN

                navsatfix_msg.status.service = NavSatStatus.SERVICE_GPS \
                                                | NavSatStatus.SERVICE_GLONASS \
                                                | NavSatStatus.SERVICE_COMPASS \
                                                | NavSatStatus.SERVICE_GALILEO
                if not gnss_fix_ok:
                    navsatfix_msg.status.status = NavSatStatus.STATUS_NO_FIX
                else:
                    if fix_type in [2, 3, 4]:  # 2D, 3D, or GNSS+DR fix
                        if carr_soln == 2:     # RTK Fixed
                            navsatfix_msg.status.status = NavSatStatus.STATUS_GBAS_FIX
                        else:
                            navsatfix_msg.status.status = NavSatStatus.STATUS_FIX
                    else:
                        navsatfix_msg.status.status = NavSatStatus.STATUS_NO_FIX
                        
                self.fix_pub.publish(navsatfix_msg)
                
            # publish NMEA sentences
            if self.publish_nmea:
                nmea = self.ublox_gnss.get_nmea()
                if nmea is not None:
                    nmea_msg = Sentence()
                    nmea_msg.header.stamp = self.get_clock().now().to_msg()
                    nmea_msg.header.frame_id = self.gnss_frame_id
                    nmea_msg.sentence = nmea
                    self.nmea_pub.publish(nmea_msg)
            
            
        except Exception as e:
            if self.verbose:
                self.get_logger().error(f"Error: {e}")
    
    
def main(args=None):
    rclpy.init(args=args)
    node = UbloxGnssNode()
    rclpy.spin(node)
    node.ublox_gnss.stop()
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()