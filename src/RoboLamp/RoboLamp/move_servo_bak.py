import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import Float32MultiArray

# Initialize a ServoKit instance to control up to 16 servos
kit = ServoKit(channels=16)

class MoveServo(Node):
    def __init__(self):
        # Initialize the node with the name 'servo_move'
        super().__init__('servo_move')
        # Create a subscription to the 'ik_angles' topic
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ik_angles',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning by referencing it

    def listener_callback(self, msg):
        # Log the received servo angles
        self.get_logger().info(f'Servo Angles: "{msg.data}"')
        # Extract angles from the message
        t1, t2, t3 = msg.data
        # Set servo 0 angle, ensuring it stays within 0 to 180 degrees
        kit.servo[0].angle = max(0, min(180, t1 + 0))
        # Set servo 1 angle, ensuring it stays within 90 to 180 degrees
        kit.servo[1].angle = max(0, min(180, max(90, t2 + 0)))  # Added constraint for safe operation
        # Set servo 3 (which connects to 15 pin of the pi) angle, ensuring it stays within 0 to 180 degrees
        kit.servo[15].angle = max(0, min(180, t3 + 0))

def main(args=None):
    rclpy.init(args=args)
    # Set default angles for servos at startup
    kit.servo[0].angle = 90
    kit.servo[1].angle = 105
    kit.servo[15].angle = 90
    # Create and start the MoveServo node
    move_servo = MoveServo()
    
    rclpy.spin(move_servo)
    # Destroy the node 
    # (Replace `minimal_subscriber` with `move_servo` for proper cleanup)
    move_servo.destroy_node()
    # Shutdown ROS
    rclpy.shutdown()

if __name__ == '__main__':
    main()
