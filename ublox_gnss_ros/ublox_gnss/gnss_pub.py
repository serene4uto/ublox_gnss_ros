import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from rtcm_msgs.msg import Message

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

        self.gps_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.rtcm_sub = self.create_subscription(Message, '/rtcm', self.onReceiveRTCMCallBack, 1)

        self.send_queue = Queue()
        self.receive_queue = Queue()
        self.stop_event = Event()

        try:
            self.get_logger().info(f"Opening serial port {SERIAL_PORT} @ {BAUDRATE}...\n")
            with GNSSSkeletonApp(
                SERIAL_PORT,
                BAUDRATE,
                TIMEOUT,
                stopevent=self.stop_event,
                sendqueue=self.send_queue,
                receivequeue=self.receive_queue,
                idonly=True,
                enableubx=True,
                showhacc=True,
                verbose=True,
            ) as gna:
                gna.run()
                sleep(2)  # wait for receiver to output at least 1 navigation solution

                while not self.stop_event.is_set():  # run until user presses CTRL-C
                    rclpy.spin_once(self)
                    # sleep(1)
                sleep(1)

        except KeyboardInterrupt:
            self.stop_event.set()

    def onReceiveRTCMCallBack(self, rtcm_msg):
        if protocol(rtcm_msg.message) == RTCM3_PROTOCOL:
            raw_data = rtcm_msg.message
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
    gnss_pub_node.destroy_node()
    rclpy.shutdown()
