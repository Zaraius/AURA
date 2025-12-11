import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class FakeMap(Node):
    def __init__(self):
        super().__init__('fake_map_publisher')
        self.publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.timer = self.create_timer(1.0, self.publish_map)
        self.get_logger().info("Publishing Fake Map starting at (0,0) ...")

    def publish_map(self):
        grid = OccupancyGrid()
        grid.header.frame_id = "odom"
        grid.header.stamp = self.get_clock().now().to_msg()
        
        # Map Settings
        grid.info.resolution = 0.1  # 10cm per pixel
        grid.info.width = 200       # 20m wide (X axis)
        grid.info.height = 200      # 20m tall (Y axis)
        
        # --- CHANGED HERE ---
        # Set origin to 0.0. This means Grid Index [0,0] is World (0.0, 0.0)
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = 0.0
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        
        # Fill with 0 (Free Space)
        grid.data = [0] * (grid.info.width * grid.info.height)
        
        self.publisher.publish(grid)

def main(args=None):
    rclpy.init(args=args)
    node = FakeMap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
