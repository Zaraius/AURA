import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import pyrealsense2 as rs
import numpy as np
import cv2

class DepthCamera(Node):
    def __init__(self):
        super().__init__('depth_camera')

        # --- TUNABLE PARAMETERS ---
        self.declare_parameter('ir_threshold', 245)
        self.declare_parameter('min_area', 50)
        self.declare_parameter('max_area', 10000)
        self.declare_parameter('max_distance', 6.0) 
        self.declare_parameter('min_distance', 0.1)
        # --- SETUP PUBLISHER ---
        # Publishes: x (pixel), y (pixel), z (meters)
        self.publisher_ = self.create_publisher(Point, '/target', 10)
        
        # --- REALSENSE CONFIGURATION ---
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Stream Infrared (Left) and Depth
        self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # Start Camera
        try:
            self.profile = self.pipeline.start(self.config)
            self.configure_emitter()
            self.get_logger().info("Camera Started. Emitter set to Max Power.")
        except Exception as e:
            self.get_logger().error(f"Could not start camera: {e}")
            return

        # Run Loop at 30Hz
        self.timer = self.create_timer(0.033, self.process_frame)

    def configure_emitter(self):
        """Forces the IR Projector to MAX power to light up the reflector."""
        depth_sensor = self.profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 1) # ON
            
        if depth_sensor.supports(rs.option.laser_power):
            max_laser = depth_sensor.get_option_range(rs.option.laser_power).max
            depth_sensor.set_option(rs.option.laser_power, max_laser)

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        ir_frame = frames.get_infrared_frame(1)
        depth_frame = frames.get_depth_frame()
        
        if not ir_frame or not depth_frame: return

        # 1. Convert to Numpy
        ir_image = np.asanyarray(ir_frame.get_data())

        # Get Parameters (in case you want to tune live)
        thresh_val = self.get_parameter('ir_threshold').value
        min_area = self.get_parameter('min_area').value
        max_area = self.get_parameter('max_area').value
        max_dist = self.get_parameter('max_distance').value
        min_dist = self.get_parameter('min_distance').value

        # 2. Threshold
        _, mask = cv2.threshold(ir_image, thresh_val, 255, cv2.THRESH_BINARY)
        kernel = np.ones((10,10), np.uint8)

        # 3. Morphology (Connect the dots)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        
        # 4. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Sort by area, largest first
            sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
            
            for c in sorted_contours:
                area = cv2.contourArea(c)

                # Filter 1: Area
                if area < min_area or area > max_area:
                    continue

                # Calculate Center
                (x, y, w, h) = cv2.boundingRect(c)
                cx = x + w // 2
                cy = y + h // 2

                # Filter 2: Distance Check
                dist = depth_frame.get_distance(cx, cy)
                
                # Logic: Reject 0.0 (too close/blinded) and Too Far
                if dist < min_dist or dist > max_dist:
                    continue

                # --- TARGET FOUND ---
                msg = Point()
                msg.x = float(cx)
                msg.y = float(cy)
                msg.z = float(dist)
                self.publisher_.publish(msg)
                
                # We found the best target, stop looking at other contours
                break

    def __del__(self):
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    node = DepthCamera()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()