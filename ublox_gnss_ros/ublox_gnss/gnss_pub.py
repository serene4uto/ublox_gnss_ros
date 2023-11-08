import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rtcm_msgs.msg import Rtcm

from threading import Event
from queue import Queue
from time import sleep

from pyrtcm import RTCMReader
from pyubx2 import RTCM3_PROTOCOL, protocol, VALCKSUM
from pygnssutils import haversine

from .gnssapp import GNSSSkeletonApp



# SERIAL_PORT = "/dev/ttyACM0"
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
TIMEOUT = 0.1

class GNSSPub(Node):
    def __init__(self):
        super().__init__('gnss_pub')

        self.gnss_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.rtcm_sub = self.create_subscription(Rtcm, '/rtcm', self.onReceiveRTCMCallBack, 10)

        self.gnss_pub_timer = self.create_timer(0.03, self.onGNSSPubTimerCallBack)

        self.send_queue = Queue()
        self.receive_queue = Queue()
        self.stop_event = Event()

        try:
            self.get_logger().info(f"Opening serial port {SERIAL_PORT} @ {BAUDRATE}...\n")
            self.gna = GNSSSkeletonApp(
                SERIAL_PORT,
                BAUDRATE,
                TIMEOUT,
                stopevent=self.stop_event,
                sendqueue=self.send_queue,
                receivequeue=self.receive_queue,
                idonly=True,
                enableubx=True,
                showhacc=True,
                verbose=False,
            )
            self.gna.run()

        except KeyboardInterrupt:
            self.stop_event.set()

    def onGNSSPubTimerCallBack(self):
        try:
            parsed_data = self.receive_queue.get()
            if hasattr(parsed_data, "identity"):
                idy = parsed_data.identity
                if idy == 'NAV-PVT':
                    navsat_fix_msg = NavSatFix()
                    navsat_fix_msg.header.stamp = self.get_clock().now().to_msg()
                    navsat_fix_msg.header.frame_id = "gnss_sensor"
                    navsat_fix_msg.latitude = parsed_data.lat
                    navsat_fix_msg.longitude = parsed_data.lon
                    navsat_fix_msg.altitude = parsed_data.height / 1000.0 # mm to m

                    #TODO: add covariance matrix

                    self.gnss_pub.publish(navsat_fix_msg)  
        
        except Exception as e:
            self.get_logger().error(f"Error: {e}")


    def onReceiveRTCMCallBack(self, rtcm_msg):
        if protocol(rtcm_msg.rtcm_data) == RTCM3_PROTOCOL:
            raw_data = rtcm_msg.rtcm_data
            # self.get_logger().info(f"RTCM3 message received: {rtcm_msg.message}")
            parsed_data = RTCMReader.parse(
                raw_data,
                validate=VALCKSUM,
                scaling=True,
                labelmsm=True,
            )
            self.send_queue.put((raw_data, parsed_data))
    

def main(args=None):
    rclpy.init(args=args)
    gnss_pub_node = GNSSPub()
    rclpy.spin(gnss_pub_node)
    gnss_pub_node.gna.stop()
    gnss_pub_node.destroy_node()
    rclpy.shutdown()
