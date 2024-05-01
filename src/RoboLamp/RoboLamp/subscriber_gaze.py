import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import math
import numpy as np

lamp_height = 40  # Height of the lamp, used in IK calculations

# Node for subscribing to gaze data and computing inverse kinematics for servo angles
class GazeSubscriber(Node):

    def __init__(self):
        super().__init__('gaze_sub')
        # Subscribe to 'gazepoint' to receive gaze coordinates
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'gazepoint',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        # Publisher for the calculated inverse kinematic angles
        self.publisher_ = self.create_publisher(Float32MultiArray, 'ik_angles', 10)
        
    def calculateIK(self, arr):
        # Unpack the coordinates
        pX, pY, pZ = arr
        # Adjust Y coordinate for robot-specific frame
        pY = 50 - pY  # Adjust the y-coordinate to ensure non-negative values

        # Calculate theta 1 (t1), angle for the base servo
        t1 = math.degrees(np.arctan2(pY, pX))  # Base rotation angle

        # Scale theta 1 to fit into the physical range of the servo
        t1 *= 180/135

        t2 = 110  # Static value for theta 2, replace with dynamic calculation if necessary

        # Calculate theta 3 (t3), considering the geometry of the lamp
        alpha = t2 - 90
        b = 90 - alpha
        w = lamp_height + 18 * math.sin(math.radians(alpha))  # Vertical displacement
        r = math.sqrt((math.pow(pX, 2) + math.pow(pY, 2)))  # Horizontal displacement
        q = r - 18 * math.cos(math.radians(alpha))  # Adjusted radius for horizontal servo
        c = math.degrees(np.arctan2(q, w))  # Calculate the angle for theta 3
        t3 = (b + c) * 11/9 - 90  # Adjust scale and offset for servo

        # Package the calculated angles into a message
        ikCoords = Float32MultiArray()
        ikCoords.data = (t1, t2, t3)
        return ikCoords

    def listener_callback(self, msg):
        # Receive gaze coordinates and compute IK angles
        self.publisher_.publish(self.calculateIK(msg.data))

def main(args=None):
    rclpy.init(args=args)

    gaze_subscriber = GazeSubscriber()

    rclpy.spin(gaze_subscriber)

    # Clean up the node manually, which is optional but good practice
    gaze_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
