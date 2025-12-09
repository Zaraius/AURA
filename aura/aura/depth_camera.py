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
        # 1. Brightness: How glowing must it be? (0-255)
        self.declare_parameter('ir_threshold', 250)
        
        # 2. Size: Ignore tiny sparkles or huge windows
        self.declare_parameter('min_area', 50)
        
        # 3. Distance: Ignore reflections further than this (Meters)
        self.declare_parameter('max_distance', 6.0) 

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

    def get_safe_depth(self, depth_frame, cx, cy):
        """
        Robust depth check. If the center of the cone is '0.0' (blinded),
        it checks the immediate neighbors to find a valid value.
        """
        # 1. Check center pixel
        dist = depth_frame.get_distance(cx, cy)
        if 0.1 < dist < 10.0:
            return dist

        # 2. If center is invalid, check a small ring around it
        # (Cones often glare in the middle but are fine on the edges)
        valid_samples = []
        radius = 5
        check_points = [
            (cx-radius, cy), (cx+radius, cy), 
            (cx, cy-radius), (cx, cy+radius)
        ]
        
        for px, py in check_points:
            if 0 <= px < 640 and 0 <= py < 480:
                d = depth_frame.get_distance(px, py)
                if 0.1 < d < 10.0:
                    valid_samples.append(d)
        
        if valid_samples:
            return np.median(valid_samples)
            
        return 0.0 # Truly invalid

    def process_frame(self):
        frames = self.pipeline.wait_for_frames()
        ir_frame = frames.get_infrared_frame(1)
        depth_frame = frames.get_depth_frame()
        
        if not ir_frame or not depth_frame: return

        # Convert to numpy
        ir_image = np.asanyarray(ir_frame.get_data())

        # Get current parameter values
        thresh_val = self.get_parameter('ir_threshold').value
        min_area = self.get_parameter('min_area').value
        max_dist = self.get_parameter('max_distance').value

        # --- VISION PIPELINE ---
        # 1. Threshold
        _, mask = cv2.threshold(ir_image, thresh_val, 255, cv2.THRESH_BINARY)
        
        # 2. Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            # Sort by area, largest first
            sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)
            
            for c in sorted_contours:
                # Filter 1: Area
                if cv2.contourArea(c) < min_area:
                    continue

                # Calculate Center
                (x, y, w, h) = cv2.boundingRect(c)
                cx = x + w // 2
                cy = y + h // 2

                # Filter 2: Depth Gating
                dist = self.get_safe_depth(depth_frame, cx, cy)
                
                # Check if valid AND within range
                if dist > 0.0 and dist <= max_dist:
                    # --- TARGET FOUND ---
                    msg = Point()
                    msg.x = float(cx)
                    msg.y = float(cy)
                    msg.z = float(dist)
                    self.publisher_.publish(msg)
                    
                    # Optional: Debug Print
                    # self.get_logger().info(f"Cone at {dist:.2f}m")
                    break # Stop after finding the closest valid target

    def __del__(self):
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    node = DepthCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()