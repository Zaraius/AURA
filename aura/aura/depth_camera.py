import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
import pyrealsense2 as rs
import numpy as np
import cv2

# --- KALMAN FILTER CLASS ---
class KalmanTracker:
    def __init__(self):
        # State: [x, y, z, dx, dy, dz]
        # Measurement: [x, y, z]
        self.kf = cv2.KalmanFilter(6, 3)

        # H Matrix (Measurement)
        self.kf.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ], np.float32)

        # F Matrix (Transition/Physics)
        self.kf.transitionMatrix = np.array([
            [1, 0, 0, 1, 0, 0], 
            [0, 1, 0, 0, 1, 0], 
            [0, 0, 1, 0, 0, 1], 
            [0, 0, 0, 1, 0, 0], 
            [0, 0, 0, 0, 1, 0], 
            [0, 0, 0, 0, 0, 1]  
        ], np.float32)

        # Q and R Matrices (Tuning Knobs)
        # processNoiseCov (Q): How erratic is the motion? (Lower = smoother, Higher = more responsive)
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 0.03
        
        # measurementNoiseCov (R): How noisy is the camera?
        self.kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 0.5
        
        self.found = False
        self.missed_frames = 0
        self.max_coast_frames = 15 # Coast for ~0.5 seconds

    def update(self, measurement):
        """Correct filter with real data"""
        mes = np.array([[np.float32(measurement[0])], 
                        [np.float32(measurement[1])], 
                        [np.float32(measurement[2])]])
        self.kf.correct(mes)
        prediction = self.kf.predict()
        self.found = True
        self.missed_frames = 0
        return (int(prediction[0]), int(prediction[1]), float(prediction[2]))

    def predict(self):
        """Coast on physics (no data)"""
        if not self.found:
            return None 
            
        self.missed_frames += 1
        if self.missed_frames > self.max_coast_frames:
            self.found = False
            return None
            
        prediction = self.kf.predict()
        return (int(prediction[0]), int(prediction[1]), float(prediction[2]))

# --- ROS 2 NODE ---
class DepthCamera(Node):
    def __init__(self):
        super().__init__('depth_camera')

        # --- TUNABLE PARAMETERS ---
        self.declare_parameter('ir_threshold', 253)
        self.declare_parameter('min_area', 50)
        self.declare_parameter('max_area', 10000)
        self.declare_parameter('max_distance', 2.75) 
        self.declare_parameter('min_distance', 0.001)

        # --- PUBLISHERS ---
        self.publisher_ = self.create_publisher(Point, '/tracker/target', 10)
        self.scan_publisher_ = self.create_publisher(LaserScan, '/scan', 10)
        
        # --- TRACKER ---
        self.tracker = KalmanTracker()

        # --- REALSENSE CONFIGURATION ---
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Stream Infrared and Depth at 30 FPS
        self.config.enable_stream(rs.stream.infrared, 1, 640, 480, rs.format.y8, 15)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
        
        try:
            self.profile = self.pipeline.start(self.config)
            self.configure_emitter()
            self.get_logger().info("Camera Started. Kalman Filter Active.")
        except Exception as e:
            self.get_logger().error(f"Could not start camera: {e}")
            return

        # Kernel for morphology
        self.kernel = np.ones((10,10), np.uint8)

        # Run Loop at 30Hz
        self.timer = self.create_timer(0.033, self.process_frame)

    def configure_emitter(self):
        """Forces the IR Projector to MAX power."""
        depth_sensor = self.profile.get_device().first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 1)
            
        if depth_sensor.supports(rs.option.laser_power):
            max_laser = depth_sensor.get_option_range(rs.option.laser_power).max
            depth_sensor.set_option(rs.option.laser_power, max_laser)

    def process_frame(self):
        frames = self.pipeline.wait_for_frames(timeout_ms=1000)
        ir_frame = frames.get_infrared_frame(1)
        depth_frame = frames.get_depth_frame()
        
        if not ir_frame or not depth_frame: return

        ir_image = np.asanyarray(ir_frame.get_data())

        # Get Parameters
        thresh_val = self.get_parameter('ir_threshold').value
        min_area = self.get_parameter('min_area').value
        max_area = self.get_parameter('max_area').value
        max_dist = self.get_parameter('max_distance').value
        min_dist = self.get_parameter('min_distance').value

        # --- VISION PIPELINE ---
        # 1. Threshold
        _, mask = cv2.threshold(ir_image, thresh_val, 255, cv2.THRESH_BINARY)
        
        # 2. Morphology (Fix Starry Sky)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)
        
        # 3. Find Contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        raw_measurement = None

        if contours:
            # Sort by area
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
                
                # Filter 3: Range Check
                if dist < min_dist or dist > max_dist:
                    continue

                # Valid Target Found
                raw_measurement = (cx, cy, dist)
                break 

        # --- KALMAN FILTER UPDATE ---
        final_target = None

        if raw_measurement:
            # We see it -> Correct the filter
            final_target = self.tracker.update(raw_measurement)
        else:
            # We don't see it -> Predict (Coast)
            final_target = self.tracker.predict()
            if final_target:
                # Optional: Log warning if coasting
                pass 

        # --- PUBLISH TARGET ---
        if final_target:
            msg = Point()
            msg.x = float(final_target[0])
            msg.y = float(final_target[1])
            msg.z = float(final_target[2])
            self.publisher_.publish(msg)

        # --- PUBLISH SCAN DATA ---
        self.publish_scan(depth_frame)

    def publish_scan(self, depth_frame):
        """Publishes depth readings as a LaserScan message."""
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = "camera_depth_frame"
        
        width = 640
        height = 480
        center_row = height // 2
        
        # Extract depth values from center row
        ranges = []
        for x in range(width):
            dist = depth_frame.get_distance(x, center_row)
            ranges.append(float(dist) if dist > 0 else float(0.01))
        
        # Configure LaserScan
        scan_msg.angle_min = -np.pi / 4  # -45 deg
        scan_msg.angle_max = np.pi / 4   # +45 deg
        scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / width
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.033
        scan_msg.range_min = 0.001
        scan_msg.range_max = 10.0
        scan_msg.ranges = ranges
        
        self.scan_publisher_.publish(scan_msg)

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