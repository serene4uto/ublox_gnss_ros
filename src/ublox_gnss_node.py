import rclpy
from rclpy.node import Node



class UbloxGnssNode(Node):
    def __init__(self):
        super().__init__('ublox_gnss_node')
        self.get_logger().info('Ublox GNSS Node has been started.')
        
        
    
def main(args=None):
    rclpy.init(args=args)
    node = UbloxGnssNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()