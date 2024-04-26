import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import math
import numpy as np

lamp_height = 40

# Receives gaze data to perform math on
class GazeSubscriber(Node):

    def __init__(self):
        super().__init__('gaze_sub')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'gazepoint',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ik_angles', 10)
        
    def calculateIK(self, arr):
        pX, pY, pZ = arr
        t1 = math.degrees(np.arctan2(pX, pY)) + 90 # Constant offset
    
        t1 = math.degrees(np.arctan2(pX, pY)) + 90 # Constant offset
    
        t2 = 105 # 110 # Do proper math later
    
        # Values used for calculating t3, the last servo
        alpha = t2 - 90
        b = 90 - alpha
        w = lamp_height + 18 * math.sin(math.radians(alpha))
        r = math.sqrt((math.pow(pX, 2) + math.pow(pY, 2)))
        q = r - 18 * math.cos(math.radians(alpha))
        c = math.degrees(np.arctan2(q, w))
        t3 = (b + c)*11/9 - 90 # 20 offset??
        ikCoords = Float32MultiArray()
        ikCoords.data = (t1, t2, t3)
        self.get_logger().info('Publishing Angles: "%s"' % ikCoords.data)
        return ikCoords

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        self.publisher_.publish(self.calculateIK(msg.data))
        

def main(args=None):
    rclpy.init(args=args)

    gaze_subscriber = GazeSubscriber()

    rclpy.spin(gaze_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gaze_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()