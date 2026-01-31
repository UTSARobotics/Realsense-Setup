from evdev import InputDevice, ecodes
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray

class TaranisPublisher(Node):
    
    def __init__(self):
        super().__init__('taranis_publisher')
        
        # Publisher for motor commands
        self.pub = self.create_publisher(
            Int32MultiArray,
            'rover_motor_commands',
            10)
        
        # Connect to Taranis controller
        try:
            self.dev = InputDevice('/dev/input/event4')
            self.get_logger().info('Taranis controller connected on event4')
        except:
            self.get_logger().error('Failed to connect to Taranis on event4')
            raise
        
        # Motor values
        self.throttle = 1023  # Center position (0-2047 range)
        self.steering = 1023  # Center position
        
        # Timer to read controller and publish
        self.timer = self.create_timer(0.01, self.read_and_publish)  # 100Hz
        
        self.get_logger().info('Taranis Tank Control Active...')
    
    def read_and_publish(self):
        """Read Taranis input and publish motor commands"""
        
        # Read all available events (non-blocking)
        for event in self.dev.read():
            if event.type == ecodes.EV_ABS:
                # Update values based on stick movement
                if event.code == ecodes.ABS_X:      # Left Stick
                    self.throttle = event.value
                elif event.code == ecodes.ABS_Y:    # Right Stick
                    self.steering = event.value
        
        # --- TANK MIXING MATH ---
        # Normalize values (range 0-2047, center is 1023.5)
        v = (self.throttle - 1023.5) * -0.0978  # Speed (Inverted so Up is positive)
        w = (self.steering - 1023.5) * 0.0978   # Turn
        
        # Calculate individual side speeds
        left_motor = int(v + w)
        right_motor = int(v - w)
        
        # Constrain to -100 to 100 range
        left_motor = max(min(left_motor, 100), -100)
        right_motor = max(min(right_motor, 100), -100)
        
        # Publish to ROS2 topic
        msg = Int32MultiArray()
        msg.data = [left_motor, right_motor]
        self.pub.publish(msg)
        
        # Debug print
        self.get_logger().debug(f"L: {left_motor} | R: {right_motor}")

def main():
    rclpy.init()
    
    try:
        node = TaranisPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
