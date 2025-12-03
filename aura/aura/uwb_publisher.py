import serial
import serial.tools.list_ports
import sys
import time
import threading
from datetime import datetime
import math
from collections import deque
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

# Anchor positions (x, y in meters)
anchor_data = [
    {'distance': None, 'timestamp': None, 'status': 'Waiting...', 'x': 0.4, 'y': 0.0, 'port': '/dev/ttyUSB0'},
    {'distance': None, 'timestamp': None, 'status': 'Waiting...', 'x': 0.0, 'y': 0.4, 'port': '/dev/ttyUSB1'},
    {'distance': None, 'timestamp': None, 'status': 'Waiting...', 'x': -0.4, 'y': 0.0, 'port': '/dev/ttyUSB2'}
]

data_lock = threading.Lock()
latest_distances = [None, None, None]

# Median filter windows for each anchor
MEDIAN_WINDOW_SIZE = 7
filter_windows = [
    deque(maxlen=MEDIAN_WINDOW_SIZE),
    deque(maxlen=MEDIAN_WINDOW_SIZE),
    deque(maxlen=MEDIAN_WINDOW_SIZE)
]

def list_serial_ports():
    """List all available serial ports"""
    ports = serial.tools.list_ports.comports()
    
    print("\n=== Available Serial Ports ===")
    for p in ports:
        print(f"  {p.device} - {p.description}")
    print("==============================\n")
    
    return [p.device for p in ports]

def apply_median_filter(anchor_index, new_distance):
    """
    Apply median filter to distance measurement
    Returns filtered distance
    """
    with data_lock:
        filter_windows[anchor_index].append(new_distance)
        
        # Need at least 3 samples for meaningful median
        if len(filter_windows[anchor_index]) >= 3:
            sorted_values = sorted(filter_windows[anchor_index])
            median_idx = len(sorted_values) // 2
            return sorted_values[median_idx]
        else:
            # Not enough samples yet, return raw value
            return new_distance

def read_uwb_data(port, baudrate, anchor_index):
    """
    Read UWB distance data from a serial port
    """
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✓ Connected to Anchor {anchor_index + 1} on {port}")
        
        # Wait for connection to stabilize
        time.sleep(1)
        ser.reset_input_buffer()

        while True:
            if ser.in_waiting > 0:
                try:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    # Look for distance data (adjust parsing based on your Arduino output format)
                    if "DIST:" in line or "Distance:" in line or "distance:" in line:
                        # Extract distance value
                        dist_str = line.split("DIST:")[-1].split("Distance:")[-1].split("distance:")[-1].strip()
                        
                        # Remove unit if present (m, cm, etc.)
                        if dist_str.endswith('m'):
                            dist_str = dist_str[:-1].strip()
                        if dist_str.endswith('cm'):
                            dist_str = dist_str[:-2].strip()
                            # Convert cm to m if needed
                            # raw_dist = float(dist_str) / 100.0
                        
                        raw_dist = float(dist_str)
                        
                        # Apply median filter
                        filtered_dist = apply_median_filter(anchor_index, raw_dist)
                        
                        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                        
                        with data_lock:
                            anchor_data[anchor_index]['distance'] = filtered_dist
                            anchor_data[anchor_index]['timestamp'] = ts
                            anchor_data[anchor_index]['status'] = 'Active'
                            latest_distances[anchor_index] = filtered_dist
                        
                        # print(f"[A{anchor_index + 1}] Raw: {raw_dist:.3f}m -> Filtered: {filtered_dist:.3f}m")
                    
                    else:
                        # Print other messages for debugging
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
    Calculate tag position using trilateration from 3 anchor distances
    """
    with data_lock:
        d1, d2, d3 = latest_distances[0], latest_distances[1], latest_distances[2]
        x1, y1 = anchor_data[0]['x'], anchor_data[0]['y']
        x2, y2 = anchor_data[1]['x'], anchor_data[1]['y']
        x3, y3 = anchor_data[2]['x'], anchor_data[2]['y']

    # Need all three distances
    if None in (d1, d2, d3):
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
        if abs(denom) < 1e-10:  # Avoid division by zero
            return None

        x = (C*E - F*B) / denom
        y = (A*F - D*C) / denom

        return x, y
    except Exception as e:
        print(f"Trilateration error: {e}")
        return None


class UWBLocalization(Node):
    def __init__(self):
        super().__init__('uwb_localization')

        # Publisher for tag position (x, y)
        self.position_pub = self.create_publisher(Float64MultiArray, '/uwb_tag_position', 10)

        # Create timer to publish position at 10Hz
        self.timer = self.create_timer(0.1, self.publish_position)

        self.get_logger().info('UWB Localization Node Started')
        self.get_logger().info(f'Median filter enabled with window size: {MEDIAN_WINDOW_SIZE}')
        self.get_logger().info(f'Publishing to topic: /uwb_tag_position')

    def publish_position(self):
        pos = trilateration()
        if pos:
            msg = Float64MultiArray()
            msg.data = [pos[0], pos[1]]
            
            self.position_pub.publish(msg)
            self.get_logger().info(f"Tag Position: x={pos[0]:.3f}m, y={pos[1]:.3f}m")
        else:
            self.get_logger().warn("Waiting for valid distance data from all anchors...")


def main(args=None):
    print("=" * 70)
    print("UWB Multi-Anchor Localization System for ROS2")
    print("=" * 70)
    print(f"Median Filter: ENABLED (Window Size = {MEDIAN_WINDOW_SIZE})")
    print()

    # List available ports
    available_ports = list_serial_ports()

    # Check that required ports exist
    required_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2']
    missing_ports = [p for p in required_ports if p not in available_ports]
    
    if missing_ports:
        print(f"✗ ERROR: Missing ports: {missing_ports}")
        print("  Make sure all 3 UWB modules are connected.")
        sys.exit(1)

    print("✓ All 3 UWB modules detected")
    print("\nAnchor Configuration:")
    for i, anchor in enumerate(anchor_data):
        print(f"  Anchor {i + 1}: {anchor['port']} at position ({anchor['x']:.1f}m, {anchor['y']:.1f}m)")
    print()

    baudrate = 115200

    # Start UWB reading threads for each anchor
    threads = []
    for i in range(3):
        thread = threading.Thread(
            target=read_uwb_data,
            args=(anchor_data[i]['port'], baudrate, i),
            daemon=True
        )
        threads.append(thread)
        thread.start()

    # Give threads time to connect
    time.sleep(2)

    # Initialize ROS2 node
    print("Starting ROS2 node...")
    rclpy.init(args=args)
    node = UWBLocalization()
    
    try:
        print("\n✓ System running! Press Ctrl+C to stop.\n")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✓ Shutdown complete")


if __name__ == "__main__":
    main()
