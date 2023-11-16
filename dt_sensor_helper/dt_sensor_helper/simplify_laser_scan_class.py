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

        self._laser_scan_subscriber = self.create_subscription(LaserScan, 
                                                               "/scan", 
                                                               self.laser_scan_callback, 
                                                               10)
        self._obstacle_detected_publisher = self.create_publisher(ObstacleDetected, 
                                                                  "/obstacle_detection", 
                                                                  10)
        
        self._obstacle_detected_range = 0.25

    def laser_scan_callback(self, laser_scan_msg: LaserScan) -> None:
    
        obstacle_detected_msgs: ObstacleDetected = ObstacleDetected()
        obstacle_detected_msgs.front = laser_scan_msg.ranges[0] < self._obstacle_detected_range
        obstacle_detected_msgs.front_left = laser_scan_msg.ranges[200] < self._obstacle_detected_range
        obstacle_detected_msgs.left = laser_scan_msg.ranges[400] < self._obstacle_detected_range
        obstacle_detected_msgs.back_left = laser_scan_msg.ranges[600] < self._obstacle_detected_range
        obstacle_detected_msgs.back = laser_scan_msg.ranges[800] < self._obstacle_detected_range
        obstacle_detected_msgs.back_right = laser_scan_msg.ranges[1000] < self._obstacle_detected_range
        obstacle_detected_msgs.right = laser_scan_msg.ranges[1200] < self._obstacle_detected_range
        obstacle_detected_msgs.front_right = laser_scan_msg.ranges[1400] < self._obstacle_detected_range

        obstacle_detected_msgs.obstacle_detected = [obstacle_detected_msgs.front,
                                                    obstacle_detected_msgs.front_left,
                                                    obstacle_detected_msgs.left,
                                                    obstacle_detected_msgs.back_left,
                                                    obstacle_detected_msgs.back,
                                                    obstacle_detected_msgs.back_right,
                                                    obstacle_detected_msgs.right,
                                                    obstacle_detected_msgs.front_right]
        
        self._obstacle_detected_publisher.publish(obstacle_detected_msgs)

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

    simplify_laser_scan_node.get_logger().info("simplify_laser_scan node is started.")
    
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