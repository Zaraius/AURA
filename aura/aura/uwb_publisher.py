import sys
import os
os.environ['MPLBACKEND'] = 'Qt5Agg'

import serial
import serial.tools.list_ports
import time
import threading
from datetime import datetime
import math
from collections import deque
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

import matplotlib
matplotlib.use('Qt5Agg', force=True)
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Anchor positions (x, y in meters)
anchor_data = [
    {'distance': None, 'timestamp': None, 'status': 'Waiting...', 'x': 0.4, 'y': 0.0, 'port': '/dev/ttyUSB0'},
    {'distance': None, 'timestamp': None, 'status': 'Waiting...', 'x': 0.0, 'y': 0.4, 'port': '/dev/ttyUSB1'},
    {'distance': None, 'timestamp': None, 'status': 'Waiting...', 'x': -0.4, 'y': 0.0, 'port': '/dev/ttyUSB2'}
]

data_lock = threading.Lock()
latest_distances = [None, None, None]
raw_distances = [None, None, None]  # Store raw unfiltered distances

# === IMPROVED FILTERING PARAMETERS ===

# Median filter windows
MEDIAN_WINDOW_SIZE = 5  # Reduced from 7 for faster response
filter_windows = [
    deque(maxlen=MEDIAN_WINDOW_SIZE),
    deque(maxlen=MEDIAN_WINDOW_SIZE),
    deque(maxlen=MEDIAN_WINDOW_SIZE)
]

# Kalman Filter for each anchor distance
class KalmanFilter1D:
    """Simple 1D Kalman filter for distance measurements"""
    def __init__(self, process_variance=0.001, measurement_variance=0.01, initial_value=1.0):
        self.process_variance = process_variance  # Q - how much we expect distance to change
        self.measurement_variance = measurement_variance  # R - sensor noise
        self.estimate = initial_value
        self.error_covariance = 1.0
        self.initialized = False
    
    def update(self, measurement):
        if not self.initialized:
            self.estimate = measurement
            self.initialized = True
            return self.estimate
        
        # Prediction step
        predicted_estimate = self.estimate
        predicted_error_cov = self.error_covariance + self.process_variance
        
        # Update step
        kalman_gain = predicted_error_cov / (predicted_error_cov + self.measurement_variance)
        self.estimate = predicted_estimate + kalman_gain * (measurement - predicted_estimate)
        self.error_covariance = (1 - kalman_gain) * predicted_error_cov
        
        return self.estimate
    
    def reset(self):
        self.initialized = False
        self.error_covariance = 1.0

# Kalman filters for each anchor
kalman_filters = [
    KalmanFilter1D(process_variance=0.002, measurement_variance=0.015),
    KalmanFilter1D(process_variance=0.002, measurement_variance=0.015),
    KalmanFilter1D(process_variance=0.002, measurement_variance=0.015)
]

# Adaptive outlier rejection
INITIAL_MAX_JUMP = 0.5
ADAPTIVE_MAX_JUMP = [INITIAL_MAX_JUMP, INITIAL_MAX_JUMP, INITIAL_MAX_JUMP]
previous_distances = [None, None, None]
stable_readings_count = [0, 0, 0]

# Rolling statistics for drift detection
STATS_WINDOW_SIZE = 50
distance_stats = [
    deque(maxlen=STATS_WINDOW_SIZE),
    deque(maxlen=STATS_WINDOW_SIZE),
    deque(maxlen=STATS_WINDOW_SIZE)
]

# Position Kalman Filter (2D)
class KalmanFilter2D:
    """2D Kalman filter for position (x, y)"""
    def __init__(self):
        # State: [x, y, vx, vy]
        self.state = np.array([0.0, 0.0, 0.0, 0.0])
        self.P = np.eye(4) * 1.0  # Error covariance
        
        # Process noise (how much we expect position/velocity to change)
        self.Q = np.eye(4)
        self.Q[0:2, 0:2] *= 0.01  # Position process noise
        self.Q[2:4, 2:4] *= 0.1   # Velocity process noise
        
        # Measurement noise (position measurement uncertainty)
        self.R = np.eye(2) * 0.05
        
        self.initialized = False
        self.dt = 0.1  # 10Hz update rate
    
    def predict(self):
        # State transition matrix (constant velocity model)
        F = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
        
        # Predict state
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q
    
    def update(self, measurement):
        """measurement: [x, y]"""
        if not self.initialized:
            self.state[0:2] = measurement
            self.initialized = True
            return self.state[0:2]
        
        # Measurement matrix (we only measure position, not velocity)
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ])
        
        # Predict
        self.predict()
        
        # Update
        y = measurement - H @ self.state  # Innovation
        S = H @ self.P @ H.T + self.R  # Innovation covariance
        K = self.P @ H.T @ np.linalg.inv(S)  # Kalman gain
        
        self.state = self.state + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        
        return self.state[0:2]

position_kalman = KalmanFilter2D()

# Position history
POSITION_HISTORY_SIZE = 100
position_history_x = deque(maxlen=POSITION_HISTORY_SIZE)
position_history_y = deque(maxlen=POSITION_HISTORY_SIZE)
current_position = {'x': None, 'y': None}
plot_ready = threading.Event()

# Consistency check parameters
CONSISTENCY_THRESHOLD = 0.8  # Maximum allowed residual error in meters (relaxed)
consecutive_failures = 0
MAX_CONSECUTIVE_FAILURES = 10
ENABLE_CONSISTENCY_CHECK = False  # Can be toggled on/off

def list_serial_ports():
    """List all available serial ports"""
    ports = serial.tools.list_ports.comports()
    
    print("\n=== Available Serial Ports ===")
    for p in ports:
        print(f"  {p.device} - {p.description}")
    print("==============================\n")
    
    return [p.device for p in ports]

def apply_median_filter(anchor_index, new_distance):
    """Apply median filter to distance measurement"""
    with data_lock:
        filter_windows[anchor_index].append(new_distance)
        
        if len(filter_windows[anchor_index]) >= 3:
            sorted_values = sorted(filter_windows[anchor_index])
            median_idx = len(sorted_values) // 2
            return sorted_values[median_idx]
        else:
            return new_distance

def adaptive_outlier_rejection(anchor_index, new_distance):
    """
    Adaptive outlier rejection that adjusts threshold based on signal stability
    """
    global previous_distances, stable_readings_count, ADAPTIVE_MAX_JUMP
    
    if previous_distances[anchor_index] is None:
        previous_distances[anchor_index] = new_distance
        return new_distance
    
    distance_change = abs(new_distance - previous_distances[anchor_index])
    
    # Check if this is an outlier
    if distance_change > ADAPTIVE_MAX_JUMP[anchor_index]:
        print(f"[A{anchor_index + 1}] REJECTED: Jump of {distance_change:.3f}m "
              f"(threshold: {ADAPTIVE_MAX_JUMP[anchor_index]:.3f}m)")
        stable_readings_count[anchor_index] = 0  # Reset stability counter
        return previous_distances[anchor_index]
    
    # Valid reading - update stability
    stable_readings_count[anchor_index] += 1
    
    # Adapt threshold based on stability
    if stable_readings_count[anchor_index] > 20:
        # Tighten threshold for stable signals (reduces drift)
        ADAPTIVE_MAX_JUMP[anchor_index] = max(0.15, ADAPTIVE_MAX_JUMP[anchor_index] * 0.95)
    elif distance_change > ADAPTIVE_MAX_JUMP[anchor_index] * 0.5:
        # Relax threshold if we're seeing larger (but acceptable) changes
        ADAPTIVE_MAX_JUMP[anchor_index] = min(INITIAL_MAX_JUMP, ADAPTIVE_MAX_JUMP[anchor_index] * 1.05)
    
    previous_distances[anchor_index] = new_distance
    return new_distance

def detect_drift(anchor_index, distance):
    """
    Detect systematic drift using rolling statistics
    Returns True if drift is detected
    """
    with data_lock:
        distance_stats[anchor_index].append(distance)
        
        if len(distance_stats[anchor_index]) < STATS_WINDOW_SIZE:
            return False
        
        # Calculate trend (simple linear regression slope)
        values = list(distance_stats[anchor_index])
        n = len(values)
        x = np.arange(n)
        
        # Calculate slope
        slope = (n * np.sum(x * values) - np.sum(x) * np.sum(values)) / \
                (n * np.sum(x**2) - np.sum(x)**2)
        
        # If slope is significant, we have drift
        drift_threshold = 0.005  # 5mm per reading on average
        if abs(slope) > drift_threshold:
            print(f"[A{anchor_index + 1}] DRIFT DETECTED: {slope:.6f} m/reading")
            return True
    
    return False

def check_trilateration_consistency(x, y, d1, d2, d3):
    """
    Check if calculated position is geometrically consistent with measurements
    Returns (is_consistent, max_error)
    """
    # Calculate distances from computed position to each anchor
    calc_d1 = math.sqrt((x - anchor_data[0]['x'])**2 + (y - anchor_data[0]['y'])**2)
    calc_d2 = math.sqrt((x - anchor_data[1]['x'])**2 + (y - anchor_data[1]['y'])**2)
    calc_d3 = math.sqrt((x - anchor_data[2]['x'])**2 + (y - anchor_data[2]['y'])**2)
    
    # Calculate residual errors
    error1 = abs(calc_d1 - d1)
    error2 = abs(calc_d2 - d2)
    error3 = abs(calc_d3 - d3)
    
    max_error = max(error1, error2, error3)
    avg_error = (error1 + error2 + error3) / 3
    
    is_consistent = max_error < CONSISTENCY_THRESHOLD
    
    if not is_consistent:
        print(f"CONSISTENCY: max_error={max_error:.3f}m (threshold={CONSISTENCY_THRESHOLD:.3f}m), "
              f"errors=[{error1:.3f}, {error2:.3f}, {error3:.3f}]")
        print(f"  Position: ({x:.3f}, {y:.3f})")
        print(f"  Measured distances: d1={d1:.3f}, d2={d2:.3f}, d3={d3:.3f}")
        print(f"  Calculated distances: {calc_d1:.3f}, {calc_d2:.3f}, {calc_d3:.3f}")
    
    return is_consistent, max_error

def read_uwb_data(port, baudrate, anchor_index):
    """Read UWB distance data from a serial port"""
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✓ Connected to Anchor {anchor_index + 1} on {port}")
        
        time.sleep(1)
        ser.reset_input_buffer()

        while True:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if "DIST:" in line or "Distance:" in line or "distance:" in line:
                        dist_str = line.split("DIST:")[-1].split("Distance:")[-1].split("distance:")[-1].strip()
                        
                        if dist_str.endswith('m'):
                            dist_str = dist_str[:-1].strip()
                        if dist_str.endswith('cm'):
                            dist_str = dist_str[:-2].strip()
                        
                        raw_dist = float(dist_str)
                        
                        # Store raw distance
                        with data_lock:
                            raw_distances[anchor_index] = raw_dist
                        
                        # Multi-stage filtering pipeline:
                        # 1. Adaptive outlier rejection
                        validated_dist = adaptive_outlier_rejection(anchor_index, raw_dist)
                        
                        # 2. Median filter to remove noise spikes
                        median_dist = apply_median_filter(anchor_index, validated_dist)
                        
                        # 3. Kalman filter for optimal estimation
                        filtered_dist = kalman_filters[anchor_index].update(median_dist)
                        
                        # 4. Check for drift
                        if detect_drift(anchor_index, filtered_dist):
                            # Reset Kalman filter if significant drift detected
                            kalman_filters[anchor_index].reset()
                            print(f"[A{anchor_index + 1}] Kalman filter reset due to drift")
                        
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        
                        with data_lock:
                            anchor_data[anchor_index]['distance'] = filtered_dist
                            anchor_data[anchor_index]['timestamp'] = ts
                            anchor_data[anchor_index]['status'] = 'Active'
                            latest_distances[anchor_index] = filtered_dist
                
                    else:
                        print(f"[A{anchor_index + 1}] {line}")
                
                except ValueError as e:
                    print(f"[A{anchor_index + 1}] Parse error: {e}")
                except Exception as e:
                    print(f"[A{anchor_index + 1}] Error: {e}")
            
            time.sleep(0.01)

    except serial.SerialException as e:
        error_msg = f"Serial Error: {str(e)[:25]}"
        with data_lock:
            anchor_data[anchor_index]['status'] = error_msg
        print(f"✗ Anchor {anchor_index + 1} ({port}): {error_msg}")
    except Exception as e:
        error_msg = f"Error: {str(e)[:25]}"
        with data_lock:
            anchor_data[anchor_index]['status'] = error_msg
        print(f"✗ Anchor {anchor_index + 1} ({port}): {error_msg}")

def trilateration():
    """
    Calculate tag position using trilateration with optional consistency checking
    """
    global consecutive_failures
    
    with data_lock:
        d1, d2, d3 = latest_distances[0], latest_distances[1], latest_distances[2]
        x1, y1 = anchor_data[0]['x'], anchor_data[0]['y']
        x2, y2 = anchor_data[1]['x'], anchor_data[1]['y']
        x3, y3 = anchor_data[2]['x'], anchor_data[2]['y']

    if None in (d1, d2, d3):
        return None

    # Sanity checks
    if d1 > 10.0 or d2 > 10.0 or d3 > 10.0:
        print(f"WARNING: Unreasonable distances: d1={d1:.3f}, d2={d2:.3f}, d3={d3:.3f}")
        return None
    
    if d1 <= 0 or d2 <= 0 or d3 <= 0:
        print(f"WARNING: Invalid distances (<=0)")
        return None

    try:
        # Trilateration equations
        A = 2*(x2 - x1)
        B = 2*(y2 - y1)
        C = d1**2 - d2**2 - x1**2 + x2**2 - y1**2 + y2**2

        D = 2*(x3 - x2)
        E = 2*(y3 - y2)
        F = d2**2 - d3**2 - x2**2 + x3**2 - y2**2 + y3**2

        denom = A*E - B*D
        
        if abs(denom) < 1e-10:
            print(f"WARNING: Singular matrix, anchors may be collinear")
            consecutive_failures += 1
            return None

        x = (C*E - F*B) / denom
        y = (A*F - D*C) / denom

        # Optional consistency check (can be disabled if too strict)
        if ENABLE_CONSISTENCY_CHECK:
            is_consistent, max_error = check_trilateration_consistency(x, y, d1, d2, d3)
            
            if not is_consistent:
                consecutive_failures += 1
                if consecutive_failures > MAX_CONSECUTIVE_FAILURES:
                    print(f"WARNING: {consecutive_failures} consecutive failures")
                return None
            
            # Reset failure counter on success
            consecutive_failures = 0

        # Sanity check on result
        if abs(x) > 5.0 or abs(y) > 5.0:
            print(f"WARNING: Position out of bounds: x={x:.3f}, y={y:.3f}")
            return None

        return x, y
    except Exception as e:
        print(f"Trilateration error: {e}")
        consecutive_failures += 1
        return None


class UWBLocalization(Node):
    def __init__(self):
        super().__init__('uwb_localization')

        self.position_pub = self.create_publisher(Float64MultiArray, '/uwb_tag_position', 10)
        self.raw_pub = self.create_publisher(Float64MultiArray, '/uwb_raw_distances', 10)
        self.filtered_pub = self.create_publisher(Float64MultiArray, '/uwb_filtered_distances', 10)

        self.timer = self.create_timer(0.1, self.publish_position)

        self.get_logger().info('UWB Localization Node Started (IMPROVED)')
        self.get_logger().info(f'Kalman filtering: ENABLED')
        self.get_logger().info(f'Adaptive outlier rejection: ENABLED')
        self.get_logger().info(f'Drift detection: ENABLED')
        self.get_logger().info(f'Consistency checking: {"ENABLED" if ENABLE_CONSISTENCY_CHECK else "DISABLED"} (threshold={CONSISTENCY_THRESHOLD}m)')
        self.get_logger().info(f'Note: Set ENABLE_CONSISTENCY_CHECK=True in code if needed')

    def publish_position(self):
        # Publish raw distances
        with data_lock:
            if all(d is not None for d in raw_distances):
                raw_msg = Float64MultiArray()
                raw_msg.data = list(raw_distances)
                self.raw_pub.publish(raw_msg)
            
            if all(d is not None for d in latest_distances):
                filt_msg = Float64MultiArray()
                filt_msg.data = list(latest_distances)
                self.filtered_pub.publish(filt_msg)
        
        # Calculate and publish position
        pos = trilateration()
        if pos:
            # Apply 2D Kalman filter to position
            filtered_pos = position_kalman.update(np.array(pos))
            
            msg = Float64MultiArray()
            msg.data = [filtered_pos[0], filtered_pos[1]]
            
            self.position_pub.publish(msg)
            self.get_logger().info(f"Tag Position: x={filtered_pos[0]:.3f}m, y={filtered_pos[1]:.3f}m")
            
            with data_lock:
                current_position['x'] = filtered_pos[0]
                current_position['y'] = filtered_pos[1]
                position_history_x.append(filtered_pos[0])
                position_history_y.append(filtered_pos[1])
        else:
            self.get_logger().warn("Waiting for valid distance data...")


def update_plot(frame, ax, anchor_scatter, trajectory_line, current_pos_scatter):
    """Update the XY plane plot"""
    ax.clear()
    
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_xlabel('X Position (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (m)', fontsize=12, fontweight='bold')
    ax.set_title('UWB Tag Position - IMPROVED (Kalman + Drift Correction)', 
                 pad=20, fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_aspect('equal')
    
    # Plot anchors
    for i, anchor in enumerate(anchor_data):
        x, y = anchor['x'], anchor['y']
        ax.plot(x, y, 'r^', markersize=15, markeredgecolor='darkred', markeredgewidth=2)
        ax.text(x, y + 0.05, f'A{i+1}', fontsize=10, ha='center', color='red', fontweight='bold')
    
    # Plot trajectory
    with data_lock:
        if len(position_history_x) > 1:
            ax.plot(list(position_history_x), list(position_history_y), 'b-', 
                   alpha=0.5, linewidth=2, label='Trajectory')
            
            if current_position['x'] is not None and current_position['y'] is not None:
                x, y = current_position['x'], current_position['y']
                ax.plot(x, y, 'go', markersize=20, label='Current Position', 
                       markeredgecolor='darkgreen', markeredgewidth=2, zorder=10)
                ax.text(x, y - 0.08, f'({x:.2f}, {y:.2f})', 
                       fontsize=9, ha='center', color='green', fontweight='bold')
    
    ax.legend(loc='upper right', fontsize=10)
    return ax,


def start_visualization():
    """Start matplotlib animation"""
    print("Starting visualization window...")
    
    plt.ion()
    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111)
    
    ax.set_xlim(-1.5, 1.5)
    ax.set_ylim(-1.5, 1.5)
    ax.set_xlabel('X Position (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y Position (m)', fontsize=12, fontweight='bold')
    ax.set_title('UWB Tag Position - XY Plane', pad=20, fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.set_aspect('equal')
    
    anchor_scatter = ax.plot([], [], 'r^')[0]
    trajectory_line = ax.plot([], [], 'b-')[0]
    current_pos_scatter = ax.plot([], [], 'go')[0]
    
    plot_ready.set()
    
    ani = animation.FuncAnimation(
        fig, 
        update_plot, 
        fargs=(ax, anchor_scatter, trajectory_line, current_pos_scatter),
        interval=100,
        blit=False,
        cache_frame_data=False
    )
    
    plt.show(block=True)


def ros_spin_thread(node):
    """Run ROS2 spin in separate thread"""
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"ROS2 spin error: {e}")


def main(args=None):
    print("=" * 70)
    print("UWB Localization System - IMPROVED with Drift Correction")
    print("=" * 70)
    print(f"✓ Kalman Filtering (1D per anchor + 2D position)")
    print(f"✓ Adaptive Outlier Rejection")
    print(f"✓ Drift Detection & Correction")
    print(f"✓ Geometric Consistency Checking")
    print()

    available_ports = list_serial_ports()
    required_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']
    missing_ports = [p for p in required_ports if p not in available_ports]
    
    if missing_ports:
        print(f"✗ ERROR: Missing ports: {missing_ports}")
        sys.exit(1)

    print("✓ All 3 UWB modules detected")
    print("\nAnchor Configuration:")
    for i, anchor in enumerate(anchor_data):
        print(f"  Anchor {i + 1}: {anchor['port']} at ({anchor['x']:.1f}m, {anchor['y']:.1f}m)")
    print()

    baudrate = 115200

    threads = []
    for i in range(3):
        thread = threading.Thread(
            target=read_uwb_data,
            args=(anchor_data[i]['port'], baudrate, i),
            daemon=True
        )
        threads.append(thread)
        thread.start()

    time.sleep(2)

    print("Starting ROS2 node...")
    rclpy.init(args=args)
    node = UWBLocalization()
    
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    ros_thread.start()
    
    print("✓ System running!")
    print("✓ Opening XY plane plot window...\n")
    
    try:
        start_visualization()
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✓ Shutdown complete")


if __name__ == "__main__":
    main()
