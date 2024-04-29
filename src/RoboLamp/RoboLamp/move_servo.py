import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import Float32MultiArray

kit = ServoKit(channels=16)

class MoveServo(Node):

    def __init__(self):
        super().__init__('servo_move')
        self.t1_target = 90 # home positions
        self.t2_target = 105
        self.t3_target = 90
        # Set the angles to a default value so they are not None upon initial start
        kit.servo[0].angle = self.t1_target
        kit.servo[1].angle = self.t2_target
        kit.servo[15].angle = self.t3_target
        self.t1_err_threshold = 4
        self.t2_err_threshold = self.t3_err_threshold = 2
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ik_angles',
            self.listener_callback,
            10)
        self.servo_timer = self.create_timer(0.01, self.update_servos)
        self.subscription  # prevent unused variable warning
        self.counter = 0

    def listener_callback(self, msg):
    """Runs everytime the angles are received, sets the new target of the servos"""
        self.counter += 1
        if self.counter % 4 == 0:
            # self.get_logger().info('Servo Angles: "%s"' % msg.data)
            t1, t2, t3 = msg.data
            self.t1_target = max(0,min(180, t1 + 0))
            self.t2_target = max(0, min(180, max(90, t2 + 0))) # so it doesnt do anything stupid
            self.t3_target = max(0,min(180, t3 + 0))

    def update_servos(self):
    """Gradually rotates the servos to a specified angle for smooth movement"""
        t1 = kit.servo[0].angle
        t1_p = self.t1_target - t1
        t2 = kit.servo[1].angle
        t2_p = self.t2_target - t2
        t3 = kit.servo[15].angle
        t3_p = self.t3_target - t3
        self.get_logger().info(f"Servo Err: {t1_p}, {t2_p}, {t3_p}")
        if abs(t1_p) > self.t1_err_threshold:
            self.get_logger().info(f"Updating t1: {t1 + 0.5*t1_p}")
            kit.servo[0].angle = t1 + (t1_p/abs(t1_p))
        
        if abs(t2_p) > self.t2_err_threshold:
            kit.servo[1].angle = t2 + 0.5*t2_p

        if abs(t3_p) > self.t3_err_threshold:
            kit.servo[15].angle = t3 + 1*t3_p
        

def main(args=None):
    rclpy.init(args=args)
    # Default Angles
   
    move_servo = MoveServo()
    

    rclpy.spin(move_servo)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()