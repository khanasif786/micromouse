# Micromouse main script
# Sanket Sharma , Asif Khan
import os
import API
import floodFill
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
        self.msg.angular.z = 0
        self.flag = 0
        self.cells = [[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],
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
        self.final_cells = [[7,7],[7,8],[8,7],[8,8]]
        self.angular_speed = 0.14 # change this to change linear speed
        self.linear_speed = 0.39 # change this to change angular speed
        self.leftwall_distance = 0
        self.rightwall_distance = 0
        self.forwardwall_distance = 0
        self.odom = Odometry()
        self.coordinates = [0,0] # first x then y
        self.angle =  0
        self.direction = "north"
        self.orient = 0 # North 
        self.xy = [15,0]
        self.WallLeft = True
        self.WallRight = True
        self.WallForward = False
        self.xprev=0
        self.yprev=0
        self.error = 0
        self.perror = 0.01
        self.p = 1.0 # last commit 7  # 2.7
        self.d = 14 # last commit 250 # 16
        self.max = 0.5
        self.kp = 1.5

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
        if(self.leftwall_distance<=0.14):
            self.WallLeft = True
        else:
            self.WallLeft = False
        if(self.rightwall_distance<=0.14):
            self.WallRight = True
        else:
            self.WallRight = False
        if(self.forwardwall_distance<=0.14):
            self.WallForward = True
        else:
            self.WallForward = False

        
        #print(self.leftwall_distance,self.rightwall_distance,self.forwardwall_distance)
    
    def odom_callback(self,msg):
        self.odom = msg
        self.coordinates[0] = int((self.odom.pose.pose.position.x + 1.446 + 0.090375 )/(0.18075)) - 1 # orientation 0.08 0.005
        self.coordinates[1] = (int((-self.odom.pose.pose.position.y + 1.446  +0.090375)/(0.18075)) - 1)
        #print("real x is ",self.coordinates[0]),
        #print("real y is ",self.coordinates[1])
        if self.orient == 0:
            self.error = self.odom.pose.pose.position.x + ((self.xy[0]-7)*(0.18075) - 0.090375)
            #print("error is",self.error)
        elif self.orient == 2:
            self.error = -(self.odom.pose.pose.position.x + ((self.xy[0]-7)*(0.18075) - 0.090375))
            #print("error is",self.error)
        elif self.orient == 3:
            self.error = self.odom.pose.pose.position.y - ((8-self.xy[1])*(0.18075) - 0.090375)
            #print("error is",self.error)
        elif self.orient == 1:
            self.error = -(self.odom.pose.pose.position.y - ((8-self.xy[1])*(0.18075) - 0.090375))
            #print("error is",self.error)
    
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
    
    def GetDirection(self): #0.03 error was working first
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

    def PID(self,error):
        if(self.orient == 0):
            angle_setpoint = (error/0.08)*0.8
            #print(angle_setpoint)
            angle_having = 0-self.angle
            angle_error = angle_having - angle_setpoint
            #print(angle_error)
            self.msg.angular.z = self.kp*angle_error
        
        elif(self.orient == 2):
            angle_setpoint = (error/0.08)*0.8
            #print(angle_setpoint)
            if self.angle < 3.14159 and self.angle > 0:
                angle_having = 3.14159-self.angle
            else:
                angle_having = -3.14159-self.angle
            
            angle_error = angle_having - angle_setpoint
            #print(angle_error)
            self.msg.angular.z = self.kp*angle_error

        elif(self.orient == 1):
            angle_setpoint = (error/0.08)*0.8
            #print(angle_setpoint)
            angle_having = -1.57-self.angle
            angle_error = angle_having - angle_setpoint
            #print(angle_error)
            self.msg.angular.z = self.kp*angle_error

        else:
            angle_setpoint = (error/0.08)*0.8
            #print(angle_setpoint)
            angle_having = 1.57-self.angle
            angle_error = angle_having - angle_setpoint
            #print(angle_error)
            self.msg.angular.z = self.kp*angle_error


    # Transitions Functions 
    # Left
    def GoLeft(self):
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0
        current_direction = self.GetDirection()
        self.now = rospy.Time.now().to_sec() 
        while(rospy.Time.now().to_sec()-self.now < 1.3  ): # current_direction!="inbetween"
            self.msg.angular.z = self.angular_speed
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection()
            #print(current_direction)
        current_direction = self.GetDirection()
        while(current_direction=="inbetween"  ):
            self.msg.angular.z = self.angular_speed
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection() 
            #print(current_direction)  

        self.msg.angular.z = 0.0
        self.velocity_pub.publish(self.msg)

        self.flag = 1

    def GoForward(self):
        self.msg.linear.x = self.linear_speed
        self.msg.linear.y = 0
        self.msg.linear.z = 0
        self.msg.angular.x = 0
        self.msg.angular.y = 0 
        self.msg.angular.z = 0
        skip = 400 # 500 was working # 250 not
        if(self.orient == 0 or self.orient == 2):
            i = 0
            self.now = rospy.Time.now().to_sec()
            while(abs(self.now-rospy.Time.now().to_sec())<0.025):
                #self.msg.angular.z = -self.p*self.error -self.d*(self.error-self.perror)
                self.PID(self.error)
                if(i%skip==0):
                    #print(-self.d*(self.error-self.perror))
                    self.perror = self.error
                self.velocity_pub.publish(self.msg)
                i = i + 1
            current_y = self.coordinates[1]
            i = 0
            while(self.coordinates[1]==current_y):
                self.PID(self.error)
                #self.msg.angular.z = -self.p*self.error -self.d*(self.error-self.perror)
                if(i%skip==0):
                    #print(-self.d*(self.error-self.perror))
                    self.perror = self.error
                self.velocity_pub.publish(self.msg)
                i = i + 1
        else:
            i = 0
            self.now = rospy.Time.now().to_sec() 
            while(abs(self.now-rospy.Time.now().to_sec())<0.025):
                #self.msg.angular.z = -self.p*self.error -self.d*(self.error-self.perror)
                self.PID(self.error)
                if(i%skip==0):
                    #print(-self.d*(self.error-self.perror))
                    self.perror = self.error
                self.velocity_pub.publish(self.msg)
                i = i + 1
            current_x = self.coordinates[0]
            i=0
            while(self.coordinates[0]==current_x):
                #self.msg.angular.z = -self.p*self.error -self.d*(self.error-self.perror)
                self.PID(self.error)
                if(i%skip==0):
                    #print(-self.d*(self.error-self.perror))
                    self.perror = self.error
                self.velocity_pub.publish(self.msg)
                i = i + 1
        # stopping
        self.perror = 0
        self.error = 0
        self.msg.angular.z = 0
        #self.msg.linear.x = -40
        #self.velocity_pub.publish(self.msg)
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
        self.now = rospy.Time.now().to_sec() 
        #self.angular_speed

        while(rospy.Time.now().to_sec()-self.now < 1.3 ): # current_direction!="inbetween"
            self.msg.angular.z = -self.angular_speed
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection()
            #print(current_direction)
        current_direction = self.GetDirection()
        while(current_direction=="inbetween"  ):
            self.msg.angular.z = -self.angular_speed
            self.velocity_pub.publish(self.msg)
            current_direction = self.GetDirection() 
            #print(current_direction)  

        self.msg.angular.z = 0.0
        self.velocity_pub.publish(self.msg)

        self.flag = 1

    #-----------------------------------------------------
    def isAccessible(self,x,y,x1,y1):
        '''returns True if mouse can move to x1,y1 from x,y (two adjacent cells)
        '''
        if (x==x1):
            if(y>y1):
                if(self.cells[y][x]==4 or self.cells[y][x]==5 or self.cells[y][x]==6 or self.cells[y][x]==10 or self.cells[y][x]==11 or self.cells[y][x]==12 or self.cells[y][x]==14 ):
                    return (False)
                else:
                    return(True)
            else:
                if(self.cells[y][x]==2 or self.cells[y][x]==7 or self.cells[y][x]==8 or self.cells[y][x]==10 or self.cells[y][x]==12 or self.cells[y][x]==13 or self.cells[y][x]==14 ):
                    return (False)
                else:
                    return(True)
                
        elif (y==y1):
            if(x>x1):
                if(self.cells[y][x]==1 or self.cells[y][x]==5 or self.cells[y][x]==8 or self.cells[y][x]==9 or self.cells[y][x]==11 or self.cells[y][x]==13 or self.cells[y][x]==14 ):
                    return (False)
                else:
                    return (True)
            else:
                if(self.cells[y][x]==3 or self.cells[y][x]==6 or self.cells[y][x]==7 or self.cells[y][x]==9 or self.cells[y][x]==11 or self.cells[y][x]==12 or self.cells[y][x]==13 ):
                    return (False)
                else:
                    return (True) 
    #-----------------------------------------------------
    def getSurrounds(self,x,y):
        ''' returns x1,y1,x2,y2,x3,y3,x4,y4 the four surrounding square
        '''
        x3= x-1
        y3=y
        x0=x
        y0=y+1
        x1=x+1
        y1=y
        x2=x
        y2=y-1
        if(x1>=16):
            x1=-1
        if(y0>=16):
            y0=-1
        return (x0,y0,x1,y1,x2,y2,x3,y3)  #order of cells- north,east,south,west
    #----------------------------------------------------- 
    def isConsistant(self,x,y):
        '''returns True if the value of current square is one
        greater than the minimum value in an accessible neighbor
        '''
        x0,y0,x1,y1,x2,y2,x3,y3 = self.getSurrounds(x,y)
        val= self.flood[y][x]
        minVals=[-1,-1,-1,-1]

        if (x0>=0 and y0>=0):
            if (self.isAccessible(x,y,x0,y0)):
                minVals[0]=self.flood[y0][x0]
        if (x1>=0 and y1>=0):
            if (self.isAccessible(x,y,x1,y1)):
                minVals[1]=self.flood[y1][x1]
        if (x2>=0 and y2>=0):
            if (self.isAccessible(x,y,x2,y2)):
                minVals[2]=self.flood[y2][x2]
        if (x3>=0 and y3>=0):
            if (self.isAccessible(x,y,x3,y3)):
                minVals[3]=self.flood[y3][x3]

        minCount=0
        for i in range(4):
            if minVals[i]== -1:
                pass
            elif minVals[i]== val+1 :
                pass
            elif minVals[i]== val-1 :
                minCount+=1
                pass

        #minVal= min(minVals)

        #return(minVal)
        
        if (minCount>0):
            return (True)
        else:
            return (False)
    #-----------------------------------------------------
    def makeConsistant(self,x,y):
        x0,y0,x1,y1,x2,y2,x3,y3 = self.getSurrounds(x,y)

        val= self.flood[y][x]
        minVals=[-1,-1,-1,-1]
        if (x0>=0 and y0>=0):
            if (self.isAccessible(x,y,x0,y0)):
                minVals[0]=self.flood[y0][x0]
                #if (flood[y0][x0]<minVal):
                #minVals.append(flood[y0][x0])
                    #minVal= flood[y0][x0]
        if (x1>=0 and y1>=0):
            if (self.isAccessible(x,y,x1,y1)):
                minVals[1]=self.flood[y1][x1]
                #if (flood[y1][x1]<minVal):
                #minVals.append(flood[y1][x1])
                    #minVal= flood[y1][x1]
        if (x2>=0 and y2>=0):
            if (self.isAccessible(x,y,x2,y2)):
                minVals[2]=self.flood[y2][x2]
                #if (flood[y2][x2]<minVal):
                #minVals.append(flood[y1][x1])
                    #minVal= flood[y2][x2]
        if (x3>=0 and y3>=0):
            if (self.isAccessible(x,y,x3,y3)):
                minVals[3]=self.flood[y3][x3]
                #if (flood[y3][x3]<minVal):
                #minVals.append(flood[y1][x1])
                    #minVal= flood[y3][x3]

        for i in range(4):
            if minVals[i]== -1:
                minVals[i]= 1000

        minVal= min(minVals)
        # Danger !!!!!
        self.flood[y][x]= minVal+1   
    #-----------------------------------------------------
    def isCentre(self,x,y):
        if((x==7 and y==8) or(x==7 and y == 7) or (x==8 and y==7) or(x==8 and y==8) ):
            return True
        else:
            return False
    #-----------------------------------------------------
        
    def floodFill(self,x,y,xprev,yprev):
        '''updates the flood matrix such that every square is consistant (current cell is x,y)
        '''
        if not self.isConsistant(x,y):
            self.flood[y][x]= self.flood[yprev][xprev]+1

        stack=[]
        stack.append(x)
        stack.append(y)
        x0,y0,x1,y1,x2,y2,x3,y3= self.getSurrounds(x,y)
        if(x0>=0 and y0>=0):
            if (self.isAccessible(x,y,x0,y0)):
                stack.append(x0)
                stack.append(y0)
        if(x1>=0 and y1>=0):
            if (self.isAccessible(x,y,x1,y1)):
                stack.append(x1)
                stack.append(y1)
        if(x2>=0 and y2>=0):
            if (self.isAccessible(x,y,x2,y2)):
                stack.append(x2)
                stack.append(y2)
        if(x3>=0 and y3>=0):
            if (self.isAccessible(x,y,x3,y3)):
                stack.append(x3)
                stack.append(y3)

        while (len(stack)!= 0):
            yrun= stack.pop()
            xrun= stack.pop()

            if self.isConsistant(xrun,yrun):
                pass
            else:
                self.makeConsistant(xrun,yrun)
                stack.append(xrun)
                stack.append(yrun)
                x0,y0,x1,y1,x2,y2,x3,y3= self.getSurrounds(xrun,yrun)
                if(x0>=0 and y0>=0):
                    if (self.isAccessible(xrun,yrun,x0,y0)):
                        stack.append(x0)
                        stack.append(y0)
                if(x1>=0 and y1>=0):
                    if (self.isAccessible(xrun,yrun,x1,y1)):
                        stack.append(x1)
                        stack.append(y1)
                if(x2>=0 and y2>=0):
                    if (self.isAccessible(xrun,yrun,x2,y2)):
                        stack.append(x2)
                        stack.append(y2)
                if(x3>=0 and y3>=0):
                    if (self.isAccessible(xrun,yrun,x3,y3)):
                        stack.append(x3)
                        stack.append(y3)
            #break

    #-----------------------------------------------------
    def updateWalls(self,x,y,orient,L,R,F):
        if(L and R and F):
            if (orient==0):
                self.cells[y][x]= 13
            elif (orient==1):
                self.cells[y][x]= 12
            elif (orient==2):
                self.cells[y][x]= 11
            elif (orient==3):
                self.cells[y][x]= 14

        elif (L and R and not F):
            if (orient==0 or orient== 2):
                self.cells[y][x]= 9
            elif (orient==1 or orient==3):
                self.cells[y][x]= 10

        elif (L and F and not R):
            if (orient==0):
                self.cells[y][x]= 8
            elif (orient==1):
                self.cells[y][x]= 7
            elif (orient==2):
                self.cells[y][x]= 6
            elif (orient==3):
                self.cells[y][x]= 5

        elif (R and F and not L):
            if (orient==0):
                self.cells[y][x]= 7
            elif (orient==1):
                self.cells[y][x]= 6
            elif (orient==2):
                self.cells[y][x]= 5
            elif (orient==3):
                self.cells[y][x]= 8

        elif(F):
            if (orient==0):
                self.cells[y][x]= 2
            elif (orient==1):
                self.cells[y][x]= 3
            elif (orient==2):
                self.cells[y][x]= 4
            elif (orient==3):
                self.cells[y][x]= 1

        elif(L):
            if (orient==0):
                self.cells[y][x]= 1
            elif (orient==1):
                self.cells[y][x]= 2
            elif (orient==2):
                self.cells[y][x]= 3
            elif (orient==3):
                self.cells[y][x]= 4

        elif(R):
            if (orient==0):
                self.cells[y][x]= 3
            elif (orient==1):
                self.cells[y][x]= 4
            elif (orient==2):
                self.cells[y][x]= 1
            elif (orient==3):
                self.cells[y][x]= 2

        else:
            self.cells[y][x]= 15

        '''if self.leftwall_distance > 0.16 and self.leftwall_distance < 0.96:
            temp = int(self.leftwall_distance/0.18075)
            if (orient==0):
                self.cells[y][x-temp] = 1
            elif (orient==2):
                self.cells[y][x+temp] = 3
            elif (orient==1):
                self.cells[y+temp][x] = 2
            else:
                self.cells[y-temp][x] = 4
        if self.rightwall_distance > 0.16 and self.rightwall_distance < 0.96:
            temp = int(self.rightwall_distance/0.18075)
            if (orient==0):
                self.cells[y][x+temp] = 3
            elif (orient==2):
                self.cells[y][x-temp] = 1
            elif (orient==1):
                self.cells[y-temp][x] = 4
            else:
                self.cells[y+temp][x] = 2
        if self.forwardwall_distance > 0.16 and self.forwardwall_distance < 0.96:
            temp = int(self.forwardwall_distance/0.18075)
            if (orient==0):
                self.cells[y+temp][x] = 2
            elif (orient==2):
                self.cells[y-temp][x] = 4
            elif (orient==1):
                self.cells[y][x+temp] = 3
            else:
                self.cells[y][x-temp] = 1'''
        


    #---------------------------------------------------------------

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
        if(self.WallForward):
            cost_f = 1001
        if(self.WallLeft):
            cost_l = 1001
        if(self.WallRight):
            cost_r = 1001 
        if(cost_f<=cost_r and cost_f<=cost_l and cost_f<=cost_b):
            return "F"
        elif(cost_l<=cost_r and cost_l<=cost_f and cost_l<=cost_b):
            return "L"
        elif(cost_r<=cost_f and cost_r<=cost_l and cost_r<=cost_b):
            return "R"
        elif(cost_b<=cost_f and cost_b<=cost_l and cost_b<=cost_r):
            return "B"
        else:
            return "F"

    #------------------------------------------------------------
    def toMove(self,x,y,xprev,yprev,orient):
        '''returns the direction to turn into L,F,R or B
        '''
        x0,y0,x1,y1,x2,y2,x3,y3 = self.getSurrounds(x,y)
        val= self.flood[y][x]
        prev=0
        minVals=[1000,1000,1000,1000]

        if (self.isAccessible(x,y,x0,y0)):
            if (x0==xprev and y0==yprev):
                prev=0
            minVals[0]= self.flood[y0][x0]

        if (self.isAccessible(x,y,x1,y1)):
            if (x1==xprev and y1==yprev):
                prev=1
            minVals[1]= self.flood[y1][x1]

        if (self.isAccessible(x,y,x2,y2)):
            if (x2==xprev and y2==yprev):
                prev=2
            minVals[2]= self.flood[y2][x2]

        if (self.isAccessible(x,y,x3,y3)):
            if (x3==xprev and y3==yprev):
                prev=3
            minVals[3]= self.flood[y3][x3]

        minVal=minVals[0]
        minCell=0
        noMovements=0
        for i in minVals:
            if (i!=1000):
                noMovements+=1

        '''for i in range(4):
            if (minVals[i]<minVal):
                minVal= minVals[i]
                minCell= i'''

        for i in range(4):
            if (minVals[i]<minVal):
                if (noMovements==1):
                    minVal= minVals[i]
                    minCell= i
                else:
                    if(i==prev):
                        pass
                    else:
                        minVal= minVals[i]
                        minCell= i

        if (minCell==orient):
            return ('F')
        elif((minCell==orient-1) or (minCell== orient+3)):
            return('L')
        elif ((minCell==orient+1) or (minCell== orient-3)):
            return('R')
        else:
            return('B')  
    #------------------------------------------------------------------  
    def updateCoordinates(self,x,y,orient):

        if (orient==0):
            y+=1
        if (orient==1):
            x+=1
        if (orient==2):
            y-=1
        if (orient==3):
            x-=1

        return(x,y)
    #-----------------------------------------------------------------          
    def run(self):
        #print(self.GetDirection())
        '''flag  = 0
        while(flag == 0):
            self.GoLeft()
            flag = 1
        while(flag == 1):
            print(self.GetDirection())
            #print("bounded")'''
        if (self.flood[self.xy[1]][self.xy[0]]!=0):
            self.floodFill(self.xy[0],self.xy[1],self.xprev,self.yprev)
        where_to_go = self.toMove(self.xy[0],self.xy[1],self.xprev,self.yprev,self.orient)   #self.where_to_go()
        print(where_to_go)
        #print(self.xy[0],self.xy[1])
        
            
        if(where_to_go == "L"):
            self.GoLeft()
            self.orient = API.orientation(self.orient,"L")
        elif(where_to_go == "R"):
            self.GoRight()
            self.orient = API.orientation(self.orient,"R")
        elif(where_to_go == "B"):
            self.GoLeft()
            self.GoLeft()
            self.orient = API.orientation(self.orient,"B")
    
        if(True): #where_to_go == "F"
            self.GoForward()
            self.xprev = self.xy[0]
            self.yprev = self.xy[1]
            '''if self.orient == 0:
                self.xy = [self.xy[0],self.xy[1]+1]
            elif self.orient == 3:
                self.xy = [self.xy[0]+1,self.xy[1]]
            elif self.orient == 2:
                self.xy = [self.xy[0],self.xy[1]-1]
            elif self.orient == 1:
                self.xy = [self.xy[0]-1,self.xy[1]]'''
            self.xy[0],self.xy[1] = self.updateCoordinates(self.xy[0],self.xy[1],self.orient)
            
            self.updateWalls(self.xy[0],self.xy[1],self.orient,self.WallLeft,self.WallRight,self.WallForward)     
            if self.xy in self.final_cells:
                rospy.spin()
            #rospy.spin()
        '''for i in range(maze_width):
            for j in range(maze_width):
                print(str(self.flood[i][j]) + " "),
            print("\n")'''
        #rospy.sleep(0.025)
        #os.system('clear')



if __name__ == '__main__':
    bot = controller()
    # wait so that time module initializes properly
    rospy.sleep(5)
    r = rospy.Rate(25)
    while not rospy.is_shutdown():
        bot.run()
        r.sleep()
