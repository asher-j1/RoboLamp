import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit
from std_msgs.msg import Float32MultiArray

kit = ServoKit(channels=16)

class MoveServo(Node):

    def __init__(self):
        super().__init__('servo_move')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'ik_angles',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Servo Angles: "%s"' % msg.data)
        t1, t2, t3 = msg.data
        kit.servo[0].angle = max(0,min(180, t1 + 0))
        kit.servo[1].angle = max(0, min(180, max(90, t2 + 0))) # so it doesnt do anything stupid
        kit.servo[15].angle = max(0,min(180, t3 + 0))


def main(args=None):
    rclpy.init(args=args)
    # Default Angles
    kit.servo[0].angle = 90
    kit.servo[1].angle = 105
    kit.servo[15].angle = 90
    move_servo = MoveServo()
    

    rclpy.spin(move_servo)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()