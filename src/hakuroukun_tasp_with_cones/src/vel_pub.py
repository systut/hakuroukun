#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def shutdown_hook():
    """
    Function executed during node shutdown.
    Used for cleanup or logging.
    """
    rospy.loginfo("Shutting down the cmd_controller_publisher_node...")

def main():
    """
    Main function for the publisher node.
    Publishes command data to '/hakuroukun_steering_controller/cmd_controller'.
    """
    # Initialize the node
    rospy.init_node('cmd_controller_publisher_node', anonymous=True)

    # Register a shutdown hook
    rospy.on_shutdown(shutdown_hook)

    # Create a publisher to the 'cmd_controller' topic
    cmd_pub = rospy.Publisher(
        '/hakuroukun_steering_controller/cmd_controller',
        Float64MultiArray,
        queue_size=10
    )

    # Define a publishing rate in Hz (10 Hz in this example)
    rate = rospy.Rate(1)

    rospy.loginfo("cmd_controller_publisher_node started. Publishing at 10 Hz.")

    while not rospy.is_shutdown():
        # Create the message
        msg = Float64MultiArray()
        
        # Example command data: [linear_velocity, steering_angle]
        # (Or any 2D/ND command your system expects)
        msg.data = [0.2, 0.0]
        
        # Log the message data
        rospy.loginfo(f"Publishing command: {msg.data}")
        
        # Publish the message
        cmd_pub.publish(msg)
        
        # Sleep to maintain the loop rate
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        # Catch exceptions related to node shutdown
        rospy.loginfo("ROSInterruptException caught. Exiting node.")
