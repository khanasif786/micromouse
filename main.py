import rospy
from geometry_msgs.msg import Twist
import time # Topic cmd_vel
from std_msgs.msg import String
from std_msgs.msg import Float32
from std_msgs.msg import Int32
# from geometry_msgs.msg import PoseWithCovariance
from sensor_msgs.msg import LaserScan
import tf
from nav_msgs.msg import Odometry


class controller():
    # constructor here
    def __init__(self):
        # node initialized
        rospy.init_node('controller_node',anonymous=True)
        self.msg=Twist()
        self.msg.linear.x = 0
        self.msg.linear.y = 0 
        self.msg.linear.z = 0
        self.msg.angular.x = 0 
        self.msg.angular.y = 0
        self.msg.angular.z = 0.2
        self.flag = 0
        self.Array = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
                      [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]]

        self.flood = [[14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14],
                     [13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13],
                     [12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12],
                     [11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11],
                     [10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10],
                     [9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9],
                     [8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8],
                     [7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7],
                     [7,6,5,4,3,2,1,0,0,1,2,3,4,5,6,7],
                     [8,7,6,5,4,3,2,1,1,2,3,4,5,6,7,8],
                     [9,8,7,6,5,4,3,2,2,3,4,5,6,7,8,9],
                     [10,9,8,7,6,5,4,3,3,4,5,6,7,8,9,10],
                     [11,10,9,8,7,6,5,4,4,5,6,7,8,9,10,11],
                     [12,11,10,9,8,7,6,5,5,6,7,8,9,10,11,12],
                     [13,12,11,10,9,8,7,6,6,7,8,9,10,11,12,13],
                     [14,13,12,11,10,9,8,7,7,8,9,10,11,12,13,14]]

        #required varibles:
        self.laser = LaserScan()
        self.odom = Odometry()
        self.coordinates = [0,0] # first x then y
        self.angle =  0
        self.direction = "west"

        # Publishers here
        self.velocity_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        #Subscribefrs here
        rospy.Subscriber("/my_mm_robot/laser/scan",LaserScan,self.laser_callback)
        rospy.Subscriber("/odom",Odometry,self.odom_callback)

    #   callback functions  
    def laser_callback(self,msg):
        self.laser = msg
    
    def odom_callback(self,msg):
        self.odom = msg
        self.coordinates[0] = int((self.odom.pose.pose.position.x + 1.36 +0.005 +0.16)/(0.17)) - 1 # orientation 0.08 0.005
        self.coordinates[1] = -(int((self.odom.pose.pose.position.y - 1.36 -0.005 -0.16)/(0.17)) + 1)
        print("x="+str(self.coordinates[0] )),
        print("y="+str(self.coordinates[1] ))
        #print("\n")
        (x,y,self.angle) = tf.transformations.euler_from_quaternion([self.odom.pose.pose.orientation.x,
                                                                     self.odom.pose.pose.orientation.y,
                                                                     self.odom.pose.pose.orientation.z,
                                                                     self.odom.pose.pose.orientation.w])
        # pass
        #print(self.angle)
    
    def GetDirection(self):
        current_angle = self.angle
        if(abs(1.57-current_angle)<0.03):
            return str("west")
        elif(abs(0-current_angle)<0.03):
            return str("north")
        elif(abs(-1.57-current_angle)<0.03):
            return str("east")
        elif(abs(3.14-current_angle)<0.03):
            return str("south")
        elif(abs(-3.14-current_angle)<0.03):
            return str("south")
        else:
            return str("inbetween")

    # Transitions Functions 
    # Left
    def GoLeft(self):
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        current_direction = self.GetDirection()

        while(current_direction!="inbetween" ):
            self.msg.angular.z = 0.2
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection()
            print(current_direction)

        while(current_direction=="inbetween"  ):
            self.msg.angular.z = 0.2
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection() 
            print(current_direction)  

        self.msg.angular.z = 0.0
        self.velocity_pub.publish(self.msg)

        self.flag = 1

    def GoRight(self):
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        current_direction = self.GetDirection()

        while(current_direction!="inbetween" ):
            self.msg.angular.z = -0.2
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection()
            print(current_direction)

        while(current_direction=="inbetween"  ):
            self.msg.angular.z = -0.2
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection() 
            print(current_direction)  

        self.msg.angular.z = 0.0
        self.velocity_pub.publish(self.msg)

        self.flag = 1

    def run(self):

        pass

if __name__ == '__main__':
    bot = controller()
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        bot.run()
        r.sleep()
