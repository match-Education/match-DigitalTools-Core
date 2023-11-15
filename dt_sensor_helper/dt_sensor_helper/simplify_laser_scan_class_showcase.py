#!/usr/bin/env python3
from time import sleep

import rclpy
from rclpy.node import Node

#############################################################################################
# Start of student import section

from sensor_msgs.msg import LaserScan
from dt_msgs.msg import ObstacleDetected

# End of student import section
#############################################################################################

#############################################################################################
# Start of student class section
class SimplifyLaserScanNode(Node):
    def __init__(self) -> None:
        super().__init__('simplify_laser_scan')

        # Create subscriber for laser scan topic
        self.laser_scan_subscriber = self.create_subscription(LaserScan, 
                                                         "/scan",
                                                         self.laser_scan_callback,
                                                         10)
        self.obstacle_detection_publisher = self.create_publisher(ObstacleDetected,
                                                             '/obstacle_detection',
                                                             10)
    
    def laser_scan_callback(self, laser_scan_msg: LaserScan):
        obstacle_detected_msg = ObstacleDetected()
        obstacle_detected_msg.front = laser_scan_msg.ranges[0] < 0.22
        obstacle_detected_msg.front_left = laser_scan_msg.ranges[200] < 0.22
        obstacle_detected_msg.left = laser_scan_msg.ranges[400] < 0.22
        obstacle_detected_msg.back_left = laser_scan_msg.ranges[600] < 0.22
        obstacle_detected_msg.back = laser_scan_msg.ranges[800] < 0.22
        obstacle_detected_msg.back_right = laser_scan_msg.ranges[1000] < 0.22
        obstacle_detected_msg.right = laser_scan_msg.ranges[1200] < 0.22
        obstacle_detected_msg.front_right = laser_scan_msg.ranges[1400] < 0.22

        self.obstacle_detection_publisher.publish(obstacle_detected_msg)

# End of student class section
#############################################################################################

def main(args = None):
    # Initialize ROS2 node and ROS2 communication (e.g. topics)
    rclpy.init(args = args) 

    simplify_laser_scan_node: SimplifyLaserScanNode = SimplifyLaserScanNode()

    # Initial sleep to wait for everything to boot properly.
    sleep(10)

    #############################################################################################
    # Start of student code section



    # End of student code section
    #############################################################################################
    
    # Pause the programm until its killed. All tasks still will be processed in the background.
    rclpy.spin(simplify_laser_scan_node) 

    # Destroy the node explicitly, otherwise garbage collector should do it.
    simplify_laser_scan_node.destroy_node()

    # Stops all communication and should always be called last in a node before finishing.
    rclpy.shutdown()

if __name__ == "__main__":
    main()