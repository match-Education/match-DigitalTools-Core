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
# Start of student function section

def laser_scan_callback(laser_scan_msg: LaserScan):
    # Declare as global so the variable can also be used in the main function
    global saved_laser_scan_msgs 
    global obstacle_detection_publisher

    saved_laser_scan_msgs = laser_scan_msg

    obstacle_detection_msgs = ObstacleDetected()
    obstacle_detection_msgs.front = saved_laser_scan_msgs.ranges[0] < 0.22
    obstacle_detection_msgs.front_left = saved_laser_scan_msgs.ranges[200] < 0.22
    obstacle_detection_msgs.left = saved_laser_scan_msgs.ranges[400] < 0.22
    obstacle_detection_msgs.back_left = saved_laser_scan_msgs.ranges[600] < 0.22
    obstacle_detection_msgs.back = saved_laser_scan_msgs.ranges[800] < 0.22
    obstacle_detection_msgs.back_right = saved_laser_scan_msgs.ranges[1000] < 0.22
    obstacle_detection_msgs.right = saved_laser_scan_msgs.ranges[1200] < 0.22
    obstacle_detection_msgs.front_right = saved_laser_scan_msgs.ranges[1400] < 0.22

    obstacle_detection_msgs.obstacle_detected = [obstacle_detection_msgs.front, 
                                                 obstacle_detection_msgs.front_left,
                                                 obstacle_detection_msgs.left,
                                                 obstacle_detection_msgs.back_left,
                                                 obstacle_detection_msgs.back, 
                                                 obstacle_detection_msgs.back_right,
                                                 obstacle_detection_msgs.right,
                                                 obstacle_detection_msgs.front_right]

    obstacle_detection_publisher.publish(obstacle_detection_msgs)

# End of student function section
#############################################################################################

def main(args = None):
    # Initialize ROS2 node and ROS2 communication (e.g. topics)
    rclpy.init(args = args) 

    # Create ROS2 Node. This is used to create for example a topic publisher.
    simplify_laser_scan_node = Node("simplify_laser_scan")  

    #############################################################################################
    # Start of student code section

    simplify_laser_scan_node.get_logger().info("simplify_laser_scan node started.")
    
    # Declare as global as it is also needed in callback function
    global obstacle_detection_publisher
    global saved_laser_scan_msgs 

    saved_laser_scan_msgs = None # Declare laser_scan_msg and initialize it.

    # Create subscriber for laser scan topic
    laser_scan_subscriber = simplify_laser_scan_node.create_subscription(LaserScan, 
                                                                         "/scan",
                                                                         laser_scan_callback,
                                                                         10)

    obstacle_detection_publisher = simplify_laser_scan_node.create_publisher(ObstacleDetected, 
                                                                             '/obstacle_detection', 
                                                                             10)
    
    while rclpy.ok():
        # if saved_laser_scan_msgs is not None:
            # Debugging
            # simplify_laser_scan_node.get_logger().info("0: " + str(saved_laser_scan_msgs.ranges[0]))
            # simplify_laser_scan_node.get_logger().info("400: " + str(saved_laser_scan_msgs.ranges[400]))
            # simplify_laser_scan_node.get_logger().info("800: " + str(saved_laser_scan_msgs.ranges[800]))
            # simplify_laser_scan_node.get_logger().info("1200: " + str(saved_laser_scan_msgs.ranges[1200]))
            # simplify_laser_scan_node.get_logger().info(str(saved_laser_scan_msgs.angle_min))
            # simplify_laser_scan_node.get_logger().info(str(saved_laser_scan_msgs.angle_max))
            # simplify_laser_scan_node.get_logger().info(str(saved_laser_scan_msgs.angle_increment))
            # simplify_laser_scan_node.get_logger().info(str(saved_laser_scan_msgs.range_min))
            # simplify_laser_scan_node.get_logger().info(str(saved_laser_scan_msgs.range_max))
        rclpy.spin_once(simplify_laser_scan_node)
    
    # End of student code section
    #############################################################################################
    
    # Pause the programm until its killed. All tasks still will be processed in the background.
    rclpy.spin(simplify_laser_scan_node) 

    # Stops all communication and should always be called last in a node before finishing.
    rclpy.shutdown()

if __name__ == "__main__":
    main()