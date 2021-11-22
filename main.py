#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import time # Topic cmd_vel
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import LaserScan



class controller():
    # constructor here
    def __init__(self):
        # node initialised
        rospy.init_node('controller_node',anonymous=True)
        self.msg=Twist()
        self.msg.linear.x = 0
        self.msg.linear.y = 0 
        self.msg.linear.z = 0
        self.msg.angular.x = 0 
        self.msg.angular.y = 0
        self.msg.angular.z = 0.2
        #required varibles:

        # Publishers here
        self.velocity_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        #Subscribefrs here
        rospy.Subscriber("/my_mm_robot/laser/scan ",)
        
    def run(self):
        # main running function\
        self.velocity_pub.publish(self.msg)
        pass

if __name__ == '__main__':
    bot = controller()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        bot.run()
        r.sleep()
