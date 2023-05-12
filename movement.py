import rospy
from geometry_msgs.msg import Twist

class MovementNode():
    def __init__(self):
        # Initialise
        rospy.init_node('MovementNode', anonymous=False)

        # Tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")

        # What function to call when you ctrl + c
        rospy.on_shutdown(self.shutdown)

        # Create a publisher which can "talk" to TurtleBot and tell it to move
        self.cmd_vel=rospy.Publisher(
            '/cmd_vel_mux/input/teleop',
            Twist,
            queue_size=10)
        
        # Move at freq 10Hz
        r = rospy.Rate(10)

        # Twist is a datatype for velocity
        move_cmd = Twist()

        # Forward at 0.2m/s
        move_cmd.linear.x = 0.2

        # Continue until ctrl + c
        while not rospy.is_shutdown():
            # publish the velocity
            self.cmd_vel.publish(move_cmd)
            # wait for 0.1 seconds (10 Hz) and publish again
            r.sleep()

    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        
        # Default Twist has linear.x of 0 and angular.z of 0 (ie Stop)
        self.cmd_vel.publish(Twist())

        # Sleep makes sure TurtleBot receive the stop command prior to shutting down the script
        rospy.sleep(1)

if __name__ == '__main__':
    MovementNode()