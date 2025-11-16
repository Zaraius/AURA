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

anchor_data = [
    {'distance': None, 'timestamp': None, 'status': 'Waiting...', 'x': 0.0, 'y': 0.6},
    {'distance': None, 'timestamp': None, 'status': 'Waiting...', 'x': -0.3, 'y': -0.3},
    {'distance': None, 'timestamp': None, 'status': 'Waiting...', 'x': 0.3, 'y': -0.3}
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
    ports = serial.tools.list_ports.comports()
    available = [p.device for p in ports]

    print("\n=== Available Serial Ports ===")
    for i, p in enumerate(ports):
        print(f"{i}: {p.device} - {p.description}")
    print("==============================\n")
    return available

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
    try:
        ser = serial.Serial(port, baudrate, timeout=1)
        ser.reset_input_buffer()

        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode(errors='ignore').strip()
                if "DIST:" in line:
                    dist_str = line.split("DIST:")[1].strip()
                    if dist_str.endswith('m'):
                        dist_str = dist_str[:-1].strip()
                    
                    raw_dist = float(dist_str)
                    
                    # Apply median filter
                    filtered_dist = apply_median_filter(anchor_index, raw_dist)
                    
                    ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                    with data_lock:
                        anchor_data[anchor_index]['distance'] = filtered_dist
                        anchor_data[anchor_index]['timestamp'] = ts
                        anchor_data[anchor_index]['status'] = 'Active'
                        latest_distances[anchor_index] = filtered_dist
            time.sleep(0.01)

    except Exception as e:
        with data_lock:
            anchor_data[anchor_index]['status'] = f"Error: {str(e)[:25]}"

def trilateration():
    with data_lock:
        d1, d2, d3 = latest_distances[0], latest_distances[1], latest_distances[2]
        x1, y1 = anchor_data[0]['x'], anchor_data[0]['y']
        x2, y2 = anchor_data[1]['x'], anchor_data[1]['y']
        x3, y3 = anchor_data[2]['x'], anchor_data[2]['y']

    if None in (d1, d2, d3):
        return None

    try:
        A = 2*(x2 - x1)
        B = 2*(y2 - y1)
        C = d1**2 - d2**2 - x1**2 + x2**2 - y1**2 + y2**2

        D = 2*(x3 - x2)
        E = 2*(y3 - y2)
        F = d2**2 - d3**2 - x2**2 + x3**2 - y2**2 + y3**2

        denom = A*E - B*D
        if denom == 0:
            return None

        x = (C*E - F*B) / denom
        y = (A*F - D*C) / denom

        return x, y
    except:
        return None


class UWBLocalization(Node):
    def __init__(self):
        super().__init__('uwb_localization')

        # Publisher for tag position (just x and y)
        self.position_pub = self.create_publisher(Float64MultiArray, '/uwb_tag_position', 10)

        # Create timer to publish position at 10Hz
        self.timer = self.create_timer(0.1, self.publish_position)

        self.get_logger().info('UWB Localization Node Started')
        self.get_logger().info(f'Median filter enabled with window size: {MEDIAN_WINDOW_SIZE}')

    def publish_position(self):
        pos = trilateration()
        if pos:
            msg = Float64MultiArray()
            msg.data = [pos[0], pos[1]]
            
            self.position_pub.publish(msg)
            self.get_logger().info(f"Tag Position: x={pos[0]:.3f}m, y={pos[1]:.3f}m")


def main(args=None):
    print("UWB Multi-Device Monitor for ROS2")
    print(f"Median Filter: ENABLED (Window Size = {MEDIAN_WINDOW_SIZE})")

    ports = list_serial_ports()
    if len(ports) < 3:
        print("Need at least 3 UWB modules connected.")
        sys.exit(1)

    print("Enter port numbers for anchors:")
    a1 = ports[int(input("Anchor 1 port #: "))]
    a2 = ports[int(input("Anchor 2 port #: "))]
    a3 = ports[int(input("Anchor 3 port #: "))]

    baudrate = 115200

    # Start UWB reading threads
    threads = [
        threading.Thread(target=read_uwb_data, args=(a1, baudrate, 0), daemon=True),
        threading.Thread(target=read_uwb_data, args=(a2, baudrate, 1), daemon=True),
        threading.Thread(target=read_uwb_data, args=(a3, baudrate, 2), daemon=True),
    ]

    for t in threads:
        t.start()

    # Initialize ROS2 node
    rclpy.init(args=args)
    node = UWBLocalization()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()