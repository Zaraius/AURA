#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import math

class OdometryVisualizer(Node):
    def __init__(self):
        super().__init__('odometry_visualizer')
        
        # Subscribe to odometry topic
        self.subscription = self.create_subscription(
            Pose2D,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Store trajectory history
        self.max_history = 1000  # Keep last 1000 points
        self.x_history = deque(maxlen=self.max_history)
        self.y_history = deque(maxlen=self.max_history)
        
        # Current pose
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Setup plot
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.ax.set_xlabel('X Position (m)', fontsize=12)
        self.ax.set_ylabel('Y Position (m)', fontsize=12)
        self.ax.set_title('Robot Odometry Visualization', fontsize=14, fontweight='bold')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Plot elements
        self.trajectory_line, = self.ax.plot([], [], 'b-', linewidth=1, alpha=0.6, label='Trajectory')
        self.robot_dot, = self.ax.plot([], [], 'ro', markersize=12, label='Robot')
        self.heading_arrow = self.ax.arrow(0, 0, 0, 0, head_width=0.05, 
                                          head_length=0.08, fc='red', ec='red')
        
        # Origin marker
        self.ax.plot(0, 0, 'gx', markersize=15, markeredgewidth=3, label='Origin')
        
        self.ax.legend(loc='upper right')
        
        # Set initial view limits
        self.ax.set_xlim(-1, 1)
        self.ax.set_ylim(-1, 1)
        
        # Animation
        self.ani = animation.FuncAnimation(
            self.fig, 
            self.update_plot, 
            interval=50,  # Update every 50ms (20 Hz)
            blit=False
        )
        
        self.get_logger().info('Odometry Visualizer Node Started')
        self.get_logger().info('Subscribing to /odom topic...')
        
    def odom_callback(self, msg):
        """Callback function for odometry messages"""
        self.current_x = msg.x
        self.current_y = msg.y
        self.current_theta = msg.theta
        
        # Add to history
        self.x_history.append(self.current_x)
        self.y_history.append(self.current_y)
        
    def update_plot(self, frame):
        """Update the plot with new data"""
        if len(self.x_history) > 0:
            # Update trajectory line
            self.trajectory_line.set_data(list(self.x_history), list(self.y_history))
            
            # Update robot position
            self.robot_dot.set_data([self.current_x], [self.current_y])
            
            # Update heading arrow
            arrow_length = 0.15
            dx = arrow_length * math.cos(self.current_theta)
            dy = arrow_length * math.sin(self.current_theta)
            
            # Remove old arrow and create new one
            if self.heading_arrow in self.ax.patches:
                self.heading_arrow.remove()
            
            self.heading_arrow = self.ax.arrow(
                self.current_x, self.current_y, dx, dy,
                head_width=0.05, head_length=0.08, 
                fc='red', ec='red', alpha=0.8
            )
            
            # Auto-scale the plot to keep robot in view
            if len(self.x_history) > 0:
                x_data = list(self.x_history)
                y_data = list(self.y_history)
                
                x_min, x_max = min(x_data + [0]), max(x_data + [0])
                y_min, y_max = min(y_data + [0]), max(y_data + [0])
                
                # Add margin
                margin = 0.5
                x_range = max(x_max - x_min, 0.5)
                y_range = max(y_max - y_min, 0.5)
                
                self.ax.set_xlim(x_min - margin, x_max + margin)
                self.ax.set_ylim(y_min - margin, y_max + margin)
            
            # Update title with current position
            self.ax.set_title(
                f'Robot Odometry | Position: ({self.current_x:.3f}, {self.current_y:.3f}) m | ' +
                f'Heading: {math.degrees(self.current_theta):.1f}°',
                fontsize=12, fontweight='bold'
            )
        
        return self.trajectory_line, self.robot_dot, self.heading_arrow


def main(args=None):
    rclpy.init(args=args)
    
    visualizer = OdometryVisualizer()
    
    # Use a separate thread for ROS spinning to avoid blocking matplotlib
    import threading
    
    def spin_node():
        rclpy.spin(visualizer)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        visualizer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()