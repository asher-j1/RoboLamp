import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import Float32MultiArray

kit = ServoKit(channels=16)

class MoveServo(Node):
    def __init__(self):
        super().__init__('servo_move')
        # Initialize home positions for the servos. These are the default positions where the servos rest.
        self.t1_target = 90  # Target angle for servo 1
        self.t2_target = 105  # Target angle for servo 2
        self.t3_target = 90  # Target angle for servo 3
        # Set the initial angles of servos to home positions
        kit.servo[0].angle = self.t1_target
        kit.servo[1].angle = self.t2_target
        kit.servo[15].angle = self.t3_target
        # Define error thresholds for each servo to minimize unnecessary small movements
        self.t1_err_threshold = 4  # Threshold for servo 1
        self.t2_err_threshold = self.t3_err_threshold = 2  # Threshold for servos 2 and 3
        # Subscribe to the 'ik_angles' topic to receive new target angles for servos
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ik_angles',
            self.listener_callback,
            10)
        # Timer to regularly update servo positions based on target angles
        self.servo_timer = self.create_timer(0.01, self.update_servos)
        self.subscription  # Dummy reference to suppress unused variable warning
        self.counter = 0

    def listener_callback(self, msg):
        """ Callback function for updating target angles based on received message data."""
        self.counter += 1
        # Process every 4th message to avoid rapid movements or jitter
        if self.counter % 4 == 0:
            t1, t2, t3 = msg.data
            # Validate and constrain the target angles to ensure mechanical safety limits
            self.t1_target = max(0, min(180, t1))
            self.t2_target = max(0, min(180, max(90, t2)))  # Ensure t2 stays above 90
            self.t3_target = max(0, min(180, t3))

    def update_servos(self):
        """ Periodically adjust servo positions towards their target angles for smooth motion."""
        # Get current angles of servos
        t1 = kit.servo[0].angle
        t2 = kit.servo[1].angle
        t3 = kit.servo[15].angle
        # Compute proportional errors for PID control
        t1_p = self.t1_target - t1
        t2_p = self.t2_target - t2
        t3_p = self.t3_target - t3
        # Log current errors for diagnostics and debugging
        self.get_logger().info(f"Servo Err: {t1_p}, {t2_p}, {t3_p}")
        # Adjust servo angles using proportional control, if errors exceed predefined thresholds
        if abs(t1_p) > self.t1_err_threshold:
            kit.servo[0].angle = t1 + (t1_p/abs(t1_p))
        if abs(t2_p) > self.t2_err_threshold:
            kit.servo[1].angle = t2 + 0.5 * t2_p
        if abs(t3_p) > self.t3_err_threshold:
            kit.servo[15].angle = t3 + 1 * t3_p

def main(args=None):
    rclpy.init(args=args)
    move_servo = MoveServo()
    rclpy.spin(move_servo)
    move_servo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
