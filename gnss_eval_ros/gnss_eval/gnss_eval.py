import haversine
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
import logging
import os
from datetime import datetime
import pytz
import threading
import numpy as np
import pyproj
from rclpy.parameter import Parameter


LOG_PATH = '.log'

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s,%(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger(__name__)

GT_COORDS = {
    'u08':  {
        'lat': 35.99040046,
        'lon': 128.92552391,
        'alt': 96.4
    },
    'u0736': {
        'lat': 35.97237383,
        'lon': 128.93865707,
        'alt': 0.0,
    },
}

def get_utm_coordinates(latitude, longitude):
    # Define the source (WGS84) and target (UTM) coordinate systems
    wgs84 = pyproj.CRS("EPSG:4326")  # WGS84 coordinate system
    crs = pyproj.CRS("EPSG:32652")  # UTM Zone 52N coordinate system (for Daegu South Korea)
    # Create a transformer for the conversion
    transformer = pyproj.Transformer.from_crs(wgs84, crs, always_xy=True)
    utm_easting, utm_northing = transformer.transform(longitude, latitude)
    return utm_easting, utm_northing

class GnssEval(Node):
    def __init__(self):
        super().__init__('gnss_eval')
        
        # ros2 params
        self.declare_parameter('log_enable', True)
        self.declare_parameter('log_path', LOG_PATH)
        # self.declare_parameter('val_plot', False)
        self.declare_parameter('ground_truth', Parameter.Type.STRING)
        

        self.ground_truth = self.get_parameter('ground_truth').value
        self.log_enable = self.get_parameter('log_enable').value
        self.log_path = self.get_parameter('log_path').value
        # self.val_plot = self.get_parameter('val_plot').value


        self.get_logger().info(f'ground_truth: {self.ground_truth}')
        self.get_logger().info(f'log_enable: {self.log_enable}')
        self.get_logger().info(f'log_path: {self.log_path}')
        # self.get_logger().info(f'val_plot: {self.val_plot}')
                               

        # subscribe to gps/fix topic
        self.sub = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_rx_callback,
            10)

        if self.ground_truth is not None:
            gt_utm_easting, gt_utm_northing = get_utm_coordinates(  GT_COORDS[self.ground_truth]['lat'], 
                                                                    GT_COORDS[self.ground_truth]['lon'])
            self.gt_utm = {
                'utm_easting': gt_utm_easting,
                'utm_northing': gt_utm_northing
            }

        # setting up logger
        os.makedirs(self.log_path, exist_ok=True)
        kst = pytz.timezone('Asia/Seoul')
        current_time = datetime.now(pytz.utc).astimezone(kst)

        if self.log_enable:
            file_handler = logging.FileHandler(os.path.join(self.log_path, f'gnss_eval_{current_time}.csv'))
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(logging.Formatter('%(asctime)s,%(message)s'))
            logger.addHandler(file_handler)
        
        # stream_handler = logging.StreamHandler()
        # stream_handler.setLevel(logging.INFO)
        # stream_handler.setFormatter(logging.Formatter('%(asctime)s %(levelname)s %(message)s'))
        # logger.addHandler(stream_handler)
    
    def navpvt_rx_callback(self, navpvt_msg):
        logger.info(navpvt_msg.data)
        pass
        
    def gps_rx_callback(self, navsat_msg):

        utm_easting, utm_northing = get_utm_coordinates(navsat_msg.latitude, navsat_msg.longitude)

        hpe_coords = np.array(
        [   float(utm_easting) - float(self.gt_utm['utm_easting']) ,
            float(utm_northing) - float(self.gt_utm['utm_northing']) ]
        )

        hpe_dist = np.sqrt(np.sum(hpe_coords**2))

        # self.get_logger().info(f'lat: {navsat_msg.latitude}, lon: {navsat_msg.longitude}, hpe_coords: {hpe_coords}, hpe_dist: {hpe_dist} m')
        logger.info(f'lat: {navsat_msg.latitude}, lon: {navsat_msg.longitude}, hpe_coords: {hpe_coords}, hpe_dist: {hpe_dist} m')


def main(args=None):
    rclpy.init(args=args)
    gnss_eval = GnssEval()
    rclpy.spin(gnss_eval)
    gnss_eval.destroy_node()
    rclpy.shutdown()
