#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray

def shutdown_hook(pub):
    """
    Function executed during node shutdown.
    Publishes a zero velocity command before exiting.
    """
    rospy.loginfo("Shutting down the cmd_controller_publisher_node...")
    # Publish a zero velocity so the robot stops
    zero_msg = Float64MultiArray()
    zero_msg.data = [0.0, 0.0]
    pub.publish(zero_msg)
    # Give some time to ensure the message is sent
    rospy.sleep(0.5)

def main():
    """
    Main function for the publisher node.
    Publishes command data to '/cmd_controller'.
    """
    # Initialize the node
    rospy.init_node('cmd_controller_publisher_node', anonymous=True)

    # Create a publisher to the 'cmd_controller' topic
    cmd_pub = rospy.Publisher(
        '/cmd_controller',
        Float64MultiArray,
        queue_size=10
    )

    # Register a shutdown hook, passing the publisher
    rospy.on_shutdown(lambda: shutdown_hook(cmd_pub))

    # Define a publishing rate in Hz
    rate = rospy.Rate(2)  # e.g., 2 Hz

    rospy.loginfo("cmd_controller_publisher_node started. Publishing at 10 Hz.")

    while not rospy.is_shutdown():
        # Create the message
        msg = Float64MultiArray()

        # Example command data: [linear_velocity, steering_angle]
        msg.data = [0.0, -0.78]

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
        rospy.loginfo("ROSInterruptException caught. Exiting node.")
