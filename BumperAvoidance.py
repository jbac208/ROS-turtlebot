import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent

class SpinUntilBumper:
    def __init__(self):
        rospy.init_node('BumperAvoidanceNode', anonymous=False)
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

        rospy.on_shutdown(self.shutdown)
        self.spin()

    def bumper_callback(self, msg):
        if msg.PRESSED == 1:
            rospy.signal_shutdown("Bumper pressed!")
        

    def spin(self):
        rospy.loginfo("Moving...")
        twist = Twist()
        #twist.angular.z = 0.1
        twist.linear.x = 0.2
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()

    def shutdown(self):
        # Stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # Default Twist has linear.x of 0 and angular.z of 0 (ie Stop)
        self.cmd_vel_pub.publish(Twist())
        # Sleep makes sure TurtleBot receive the stop command prior to shutting down the script
        #rospy.sleep(1)

if __name__ == '__main__':
    SpinUntilBumper()
