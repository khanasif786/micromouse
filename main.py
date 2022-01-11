# Micromouse main script
# Sanket Sharma , Asif Khan

import API
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

maze_width = 16 

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

        #required variables:
        self.leftwall_distance = 0
        self.rightwall_distance = 0
        self.forwardwall_distance = 0
        self.odom = Odometry()
        self.coordinates = [0,0] # first x then y
        self.angle =  0
        self.direction = "north"
        self.orient = 0 # North 
        self.xy = [0,0]

        # Publishers here
        self.velocity_pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
        #Subscribefrs here
        rospy.Subscriber("/my_mm_robot/laser/scan",LaserScan,self.laser_callback)
        rospy.Subscriber("/odom",Odometry,self.odom_callback)

    #   callback functions  
    def laser_callback(self,msg):
        #self.laser = msg
        self.leftwall_distance = msg.ranges[359]
        self.rightwall_distance = msg.ranges[0]
        self.forwardwall_distance =msg.ranges[180]
        #print(self.leftwall_distance,self.rightwall_distance,self.forwardwall_distance)
    
    def odom_callback(self,msg):
        self.odom = msg
        self.coordinates[0] = int((self.odom.pose.pose.position.x + 1.446 + 0.090375 )/(0.18075)) - 1 # orientation 0.08 0.005
        self.coordinates[1] = int((-self.odom.pose.pose.position.y + 1.446  +0.090375)/(0.18075)) - 1
        if(self.coordinates[0] == -1):
            self.coordinates[0] = 0
        if(self.coordinates[1] == -1):
            self.coordinates[1] = 0
        #print("x="+str(self.coordinates[0] )),
        #print("y="+str(self.coordinates[1] ))
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
            #print(current_direction)

        while(current_direction=="inbetween"  ):
            self.msg.angular.z = 0.2
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection() 
            #print(current_direction)  

        self.msg.angular.z = 0.0
        self.velocity_pub.publish(self.msg)

        self.flag = 1

    def GoForward(self):
        self.msg.linear.x = 0.1
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0 
        self.msg.angular.z = 0
        if(self.orient == 0 or self.orient == 2):
            self.now = rospy.Time.now().to_sec() 
            while(abs(self.now-rospy.Time.now().to_sec())<1):
                self.velocity_pub.publish(self.msg)
            current_y = self.coordinates[1]
            while(self.coordinates[1]==current_y):
                self.velocity_pub.publish(self.msg)
        else:
            self.now = rospy.Time.now().to_sec() 
            while(abs(self.now-rospy.Time.now().to_sec())<1):
                self.velocity_pub.publish(self.msg)
            current_x = self.coordinates[0]
            while(self.coordinates[0]==current_x):
                self.velocity_pub.publish(self.msg)
        # stopping
        self.msg.linear.x = 0
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
            #print(current_direction)

        while(current_direction=="inbetween"  ):
            self.msg.angular.z = -0.2
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection() 
            #print(current_direction)  

        self.msg.angular.z = 0.0
        self.velocity_pub.publish(self.msg)

        self.flag = 1

    def where_to_go(self):
        global maze_width
        cost_f = 0
        cost_l = 0
        cost_r = 0
        cost_b = 0
        if(self.orient==0):
            if(self.xy[1]==maze_width-1):
                cost_f = 1000
            else:
                cost_f = self.flood[self.xy[0]][self.xy[1]+1]
            
            if(self.xy[1]==0):
                cost_b = 1000
            else:
                cost_b = self.flood[self.xy[0]][self.xy[1]-1]

            if(self.xy[0]==0):
                cost_r = 1000
            else:
                cost_r = self.flood[self.xy[0]-1][self.xy[1]]

            if(self.xy[0]==maze_width-1):
                cost_l = 1000
            else:
                cost_l = self.flood[self.xy[0]+1][self.xy[1]]
        #-----------------------------------------------------------
        if(self.orient==2):
            if(self.xy[1]==0):
                cost_f = 1000
            else:
                cost_f = self.flood[self.xy[0]][self.xy[1]-1]
            
            if(self.xy[1]==maze_width-1):
                cost_b = 1000
            else:
                cost_b = self.flood[self.xy[0]][self.xy[1]+1]

            if(self.xy[0]==maze_width-1):
                cost_r = 1000
            else:
                cost_r = self.flood[self.xy[0]+1][self.xy[1]]

            if(self.xy[0]==0):
                cost_l = 1000
            else:
                cost_l = self.flood[self.xy[0]-1][self.xy[1]]
        #----------------------------------------------------------------------
        if(self.orient==3):
            if(self.xy[0]==maze_width-1):
                cost_f = 1000
            else:
                cost_f = self.flood[self.xy[0]+1][self.xy[1]]
            
            if(self.xy[0]==0):
                cost_b = 1000
            else:
                cost_b = self.flood[self.xy[0]-1][self.xy[1]]

            if(self.xy[1]==maze_width-1):
                cost_r = 1000
            else:
                cost_r = self.flood[self.xy[0]][self.xy[1]+1]

            if(self.xy[1]==0):
                cost_l = 1000
            else:
                cost_l = self.flood[self.xy[0]][self.xy[1]-1]
        #---------------------------------------------------------------------
        if(self.orient==1):
            if(self.xy[0]==0):
                cost_f = 1000
            else:
                cost_f = self.flood[self.xy[0]-1][self.xy[1]]
            
            if(self.xy[0]==maze_width-1):
                cost_b = 1000
            else:
                cost_b = self.flood[self.xy[0]+1][self.xy[1]]

            if(self.xy[1]==0):
                cost_r = 1000
            else:
                cost_r = self.flood[self.xy[0]][self.xy[1]-1]

            if(self.xy[1]==maze_width-1):
                cost_l = 1000
            else:
                cost_l = self.flood[self.xy[0]][self.xy[1]+1]
        #-------------------------------------------------------------------------

        if(cost_f<=cost_r and cost_f<=cost_l and cost_f<=cost_b):
            return "F"
        elif(cost_l<=cost_r and cost_l<=cost_f and cost_l<=cost_b):
            return "L"
        elif(cost_r<=cost_f and cost_r<=cost_l and cost_r<=cost_b):
            return "R"
        else:
            return "B"

            
    def run(self):
        #print(self.GetDirection())
        where_to_go = self.where_to_go()
        print(where_to_go)
        if(where_to_go == "F"):
            self.GoForward()
            if self.orient == 0:
                self.xy = [self.xy[0],self.xy[1]+1]
            elif self.orient == 3:
                self.xy = [self.xy[0]+1,self.xy[1]]
            elif self.orient == 2:
                self.xy = [self.xy[0],self.xy[1]-1]
            elif self.orient == 1:
                self.xy = [self.xy[0]-1,self.xy[1]]

        elif(where_to_go == "L"):
            self.GoLeft()
            self.orient = API.orientation(self.orient,"L")
        elif(where_to_go == "R"):
            self.GoRight()
            self.orient = API.orientation(self.orient,"R")
        else:
            self.GoLeft()
            self.GoLeft()
            self.orient = API.orientation(self.orient,"B")

        
        if(self.flag == 0):
            self.GoForward()


if __name__ == '__main__':
    bot = controller()
    print("Current Time:",rospy.Time.now().to_sec())
    rospy.sleep(5)
    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        bot.run()
        r.sleep()
