#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file


This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
         
    
'''

import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *


class offboard_control:


    def __init__(self):
        # Initialise rosnode
        rospy.init_node('offboard_control', anonymous=True)


    
    def setArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly delacre other service proxies 

   
    def offboard_set_mode(self):
        

        rospy.wait_for_service('mavros/set_mode')  
        try:
            missionMode = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            # missionMode(base_mode=0)
            missionMode(base_mode=0,custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print ("Service mission call failed: %s"%e)
        
        pass

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
    
   
class stateMoniter:
    def __init__(self):
        self.state = State()
        self.postd=PoseStamped()
        # Instantiate a setpoints message

        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    

    def statepos(self,msg):
        self.postd.pose.position.x = msg.pose.position.x
        self.postd.pose.position.y = msg.pose.position.y
        self.postd.pose.position.z = msg.pose.position.z

def main():


    stateMt = stateMoniter()
    ofb_ctl = offboard_control()

    # Initialize publishers
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    local_vel_pub = rospy.Publisher('mavros/setpoint_velocity/cmd_vel', TwistStamped, queue_size=10)
    # Specify the rate 
    rate = rospy.Rate(20.0)

    # Make the list of setpoints 
    setpoints = [[0,0,0],[0,0,10],[10,0,10]] #List to setpoints

    # Similarly initialize other publishers 

    # Create empty message containers 
    pos =PoseStamped()
    pos.pose.position.x = 0
    pos.pose.position.y = 0
    pos.pose.position.z = 10

    # Set your velocity here
    vel = TwistStamped()
    vel.twist.linear.x = 0
    vel.twist.linear.y = 0
    vel.twist.linear.z = 0.01
    
    # Similarly add other containers 

    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    # Similarly initialize other subscribers 
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped, stateMt.statepos)


    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()


    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    while not stateMt.state.mode=="OFFBOARD":
        ofb_ctl.offboard_set_mode()
        rate.sleep()
    print ("OFFBOARD mode activated")
    p1=True
    p2=False
    p3=True
    p4=True
    p5=True
    p6=True
    forward=True
    
    
    # Publish the setpoints 
    n=0
    while not rospy.is_shutdown():
    #     '''
    #     Step 1: Set the setpoint 
    #     Step 2: Then wait till the drone reaches the setpoint, 
    #     Step 3: Check if the drone has reached the setpoint by checking the topic /mavros/local_position/pose 
    #     Step 4: Once the drone reaches the setpoint, publish the next setpoint , repeat the process until all the setpoints are done  


    #     Write your algorithm here 
    #     # '''
    #     # print(abs(stateMt.postd.pose.position.z))

        distance_tolerance=0.01
        
        local_pos_pub.publish(pos)
        local_vel_pub.publish(vel)
        rate.sleep()

        if  abs(stateMt.postd.pose.position.z-10)>= distance_tolerance:
    
        
            p2=True
        else:
            p2=False
        


        if p2==False and forward==True:
            pos.pose.position.x = 10
            pos.pose.position.y = 0
            pos.pose.position.z = 10
            vel.twist.linear.x = 0.01
            vel.twist.linear.y = 0.01
            vel.twist.linear.z = 0.01
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            rate.sleep()
            if  abs(stateMt.postd.pose.position.x-10)>= distance_tolerance:
                p3=True
            else:
                p3=False

        if p3==False and forward==True:
            pos.pose.position.x = 10
            pos.pose.position.y = 10
            pos.pose.position.z = 10
            vel.twist.linear.x = 0.01
            vel.twist.linear.y = 0.01
            vel.twist.linear.z = 0.01
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            rate.sleep()
            if  abs(stateMt.postd.pose.position.y-10)>= distance_tolerance:
                p4=True
            else:
                p4=False
                forward=False
        if p4==False and forward==False:
            pos.pose.position.x = 0
            pos.pose.position.y = 10
            pos.pose.position.z = 10
            vel.twist.linear.x = 0.01
            vel.twist.linear.y = 0.01
            vel.twist.linear.z = 0.01
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            rate.sleep()
            if  abs(stateMt.postd.pose.position.x)>= distance_tolerance:
                p5=True
            else:
                p5=False

        if p5==False and forward==False:
            pos.pose.position.x = 0
            pos.pose.position.y = 0
            pos.pose.position.z = 10
            vel.twist.linear.x = 0.01
            vel.twist.linear.y = 0.01
            vel.twist.linear.z = 0.01
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            rate.sleep()
            if  abs(stateMt.postd.pose.position.y)>= distance_tolerance:
                p6=True
            else:
                p6=False


        if p6==False and forward==False:
            pos.pose.position.x = 0
            pos.pose.position.y = 0
            pos.pose.position.z = 0
            vel.twist.linear.x = 0.01
            vel.twist.linear.y = 0.01
            vel.twist.linear.z = 0.01
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            rate.sleep()
            
        
            


        # print("loop no : ")
        # n=n+1
        # print(n)
        # print("\n")

            
       
        
        
            
        
        # print("x=")
        # print(stateMt.postd.pose.position.x)
        # print("y=")
        # print(stateMt.postd.pose.position.y)
        # print("z=")
        # print(stateMt.postd.pose.position.z)
        # print("\n")
       

        # print("reaching to altitude of 10!!!!!!!")
        # if abs(stateMt.postd.pose.position.z-10) >= distance_tolerance :
        #     z=False
        # else :
        #     z=True
        #     break 
                
                
            
          
        # if(z==True):

        #     pos.pose.position.x = 10
        #     pos.pose.position.y = 0
        #     pos.pose.position.z = 10
        #     vel.twist.linear.x = 0.01
        #     vel.twist.linear.y = 0
        #     vel.twist.linear.z = 0
        #     local_pos_pub.publish(pos)
        #     local_vel_pub.publish(vel)
            

        #     print("reaching to altitude of 10!!!!!!!")
        #     if abs(stateMt.postd.pose.position.x-10) >= distance_tolerance :
        #         z=False
        #     else :
        #         z=True
        #         break         

        # pos.pose.position.x = 10
        # pos.pose.position.y = 10
        # pos.pose.position.z = 10
        # vel.twist.linear.x = 0
        # vel.twist.linear.y = 5
        # vel.twist.linear.z = 0
        # local_pos_pub.publish(pos)
        # local_vel_pub.publish(vel)
        # rate.sleep()
        # while abs(stateMt.postd.pose.position.x-10) >= distance_tolerance and abs(stateMt.postd.pose.position.y-10)>= distance_tolerance and abs(stateMt.postd.pose.position.z-10)>= distance_tolerance:
        #     # Porportional controller.

        #     local_pos_pub.publish(pos)
        #     local_vel_pub.publish(vel)
        #     rate.sleep()

        # pos.pose.position.x = 0
        # pos.pose.position.y = 10
        # pos.pose.position.z = 10
        # local_pos_pub.publish(pos)
        # local_vel_pub.publish(vel)
        # rate.sleep()
        # while (stateMt.postd.pose.position.x-10) >= distance_tolerance:
        #     # Porportional controller.

        #     local_pos_pub.publish(pos)
        #     local_vel_pub.publish(vel)
        #     rate.sleep()

        # pos.pose.position.x = 0
        # pos.pose.position.y = 0
        # pos.pose.position.z = 10
        # local_pos_pub.publish(pos)
        # local_vel_pub.publish(vel)
        # rate.sleep()
        # while (stateMt.postd.pose.position.y-10) >= distance_tolerance:
        #     # Porportional controller.

        #     local_pos_pub.publish(pos)
        #     local_vel_pub.publish(vel)
        #     rate.sleep()

        # pos.pose.position.x = 0
        # pos.pose.position.y = 0
        # pos.pose.position.z = 0
        # local_pos_pub.publish(pos)
        # local_vel_pub.publish(vel)
        # rate.sleep()
        # while (stateMt.postd.pose.position.z-10) >= distance_tolerance:
        #     # Porportional controller.

        #     local_pos_pub.publish(pos)
        #     local_vel_pub.publish(vel)
        #     rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass