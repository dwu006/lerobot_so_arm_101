#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
import time

class JointStateMonitor(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')
        
        # Subscribe to joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher for enhanced joint states (with velocity/acceleration)
        self.enhanced_pub = self.create_publisher(
            JointState,
            '/joint_states_enhanced',
            10
        )
        
        # Store previous state for velocity calculation
        self.prev_positions = None
        self.prev_time = None
        self.prev_velocities = None
        
        self.get_logger().info('Joint State Monitor started')
        self.get_logger().info('Subscribing to: /joint_states')
        self.get_logger().info('Publishing to: /joint_states_enhanced')

    def joint_state_callback(self, msg):
        current_time = time.time()
        
        # Create enhanced message
        enhanced_msg = JointState()
        enhanced_msg.header = Header()
        enhanced_msg.header.stamp = self.get_clock().now().to_msg()
        enhanced_msg.name = msg.name
        enhanced_msg.position = msg.position
        
        # Calculate velocities if we have previous data
        if self.prev_positions is not None and self.prev_time is not None:
            dt = current_time - self.prev_time
            if dt > 0:
                velocities = []
                for i, (curr_pos, prev_pos) in enumerate(zip(msg.position, self.prev_positions)):
                    vel = (curr_pos - prev_pos) / dt
                    velocities.append(vel)
                enhanced_msg.velocity = velocities
                
                # Calculate accelerations if we have previous velocities
                if self.prev_velocities is not None:
                    accelerations = []
                    for curr_vel, prev_vel in zip(velocities, self.prev_velocities):
                        acc = (curr_vel - prev_vel) / dt
                        accelerations.append(acc)
                    enhanced_msg.effort = accelerations  # Using effort field for acceleration
                
                self.prev_velocities = velocities
        
        # Store current state for next calculation
        self.prev_positions = list(msg.position)
        self.prev_time = current_time
        
        # Publish enhanced state
        self.enhanced_pub.publish(enhanced_msg)
        
        # Log current state (optional - can be removed for performance)
        if len(msg.position) >= 6:
            self.get_logger().info(
                f'Joint States - Pos: [{msg.position[0]:.3f}, {msg.position[1]:.3f}, '
                f'{msg.position[2]:.3f}, {msg.position[3]:.3f}, {msg.position[4]:.3f}, {msg.position[5]:.3f}]'
            )

def main(args=None):
    rclpy.init(args=args)
    monitor = JointStateMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
