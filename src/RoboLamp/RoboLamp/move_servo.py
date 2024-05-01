import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import Float32MultiArray

kit = ServoKit(channels=16)

class MoveServo(Node):
    def __init__(self):
        super().__init__('servo_move')
        # Initial home positions for the servos
        self.t1_target = 90
        self.t2_target = 105
        self.t3_target = 90
        # Set initial servo angles to home positions 
        kit.servo[0].angle = self.t1_target
        kit.servo[1].angle = self.t2_target
        kit.servo[15].angle = self.t3_target
        # Error thresholds to control servo movement sensitivity
        self.t1_err_threshold = 4
        self.t2_err_threshold = self.t3_err_threshold = 2
        # Subscribe to 'ik_angles' topic to receive target angles for servos
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ik_angles',
            self.listener_callback,
            10)
        # Timer to update servo positions at regular intervals
        self.servo_timer = self.create_timer(0.01, self.update_servos)
        self.subscription  # prevent unused variable warning
        self.counter = 0

    def listener_callback(self, msg):
        """Callback function that runs every time new angle data is received. Updates target angles for servo movements."""
        self.counter += 1
        if self.counter % 4 == 0:  # Update targets every 4th message to avoid jitter
            t1, t2, t3 = msg.data
            self.t1_target = max(0, min(180, t1))
            self.t2_target = max(0, min(180, max(90, t2)))  # Ensure t2 doesn't go below 90
            self.t3_target = max(0, min(180, t3))

    def update_servos(self):
        """Adjusts servo angles gradually towards the target angles for smoother motion."""
        t1 = kit.servo[0].angle
        t2 = kit.servo[1].angle
        t3 = kit.servo[15].angle
        # Calculate proportional error for each servo
        t1_p = self.t1_target - t1
        t2_p = self.t2_target - t2
        t3_p = self.t3_target - t3
        # Log the current error values for debugging
        self.get_logger().info(f"Servo Err: {t1_p}, {t2_p}, {t3_p}")
        # Update servo angle based on proportional control if error exceeds threshold
        if abs(t1_p) > self.t1_err_threshold:
            kit.servo[0].angle = t1 + (t1_p/abs(t1_p))  # Move servo in the direction of error reduction
        if abs(t2_p) > self.t2_err_threshold:
            kit.servo[1].angle = t2 + 0.5 * t2_p
        if abs(t3_p) > self.t3_err_threshold:
            kit.servo[15].angle = t3 + 1 * t3_p

def main(args=None):
    rclpy.init(args=args)
    move_servo = MoveServo()
    rclpy.spin(move_servo)
    # Replace `minimal_subscriber` with `move_servo` 
    move_servo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
