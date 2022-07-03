#!/usr/bin/env python3


'''
This is a boiler plate script that contains hint about different services that are to be used
to complete the task.
Use this code snippet in your code or you can also continue adding your code in the same file

s
This python file runs a ROS-node of name offboard_control which controls the drone in offboard mode. 
See the documentation for offboard mode in px4 here() to understand more about offboard mode 
This node publishes and subsribes the following topics:

	 Services to be called                   Publications                                          Subscriptions				
	/mavros/cmd/arming                       /mavros/setpoint_position/local                       /mavros/state
    /mavros/set_mode                         /mavros/setpoint_velocity/cmd_vel                     /mavros/local_position/pose   
    /activate_gripper                                                                              /gripper_check
    
'''

from geometry_msgs import msg
import rospy
from geometry_msgs.msg import *
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import *
from gazebo_ros_link_attacher.srv import *
from std_srvs.srv import *
# std_srvs/Empty


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
    
    def disArm(self):
        # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
        rospy.wait_for_service('mavros/cmd/arming')  # Waiting untill the service starts 
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            armService(False)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s"%e)

        # Similarly delacre other service proxies

   
    def offboard_set_mode(self):
        

        rospy.wait_for_service('mavros/set_mode')  
        try:
            missionMode = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            missionMode(base_mode=0,custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            print ("Service mission call failed: %s"%e)


    def land_mode(self):
        

        rospy.wait_for_service('mavros/set_mode')  
        try:
            missionMode = rospy.ServiceProxy('mavros/set_mode', mavros_msgs.srv.SetMode) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
            missionMode(base_mode=0,custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
            print ("Service mission call failed: %s"%e)

    

    def setGrip_on_object(self):
        

        rospy.wait_for_service('activate_gripper')  
        try:
            Gripped= rospy.ServiceProxy('activate_gripper',Gripper)
            
            Gripped(activate_gripper=True)
            print("hello )):")

        except rospy.ServiceException as e:
            print ("Drone is not placed properly ;( !!!!: %s"%e)
        print("hey i reached here ")

    def looseGrip_on_object(self):

        rospy.wait_for_service('activate_gripper')  
        try:
            Gripped= rospy.ServiceProxy('activate_gripper',Gripper)
            
            Gripped(activate_gripper=False)
            print("Now I have ungripped at given position :) ")
        except rospy.ServiceException as e:
            print ("Drone is not placed properly ;( !!!!: %s"%e)
        
        

        # Call /mavros/set_mode to set the mode the drone to OFFBOARD
        # and print fail message on failure
    
   
class stateMoniter:
    def __init__(self):
        self.state = State()
        self.postd=PoseStamped()
        self.posstring=String()

        # Instantiate a setpoints message

        
    def stateCb(self, msg):
        # Callback function for topic /mavros/state
        self.state = msg

    # Create more callback functions for other subscribers    

    def statepos(self,msg):
        self.postd.pose.position.x = msg.pose.position.x
        self.postd.pose.position.y = msg.pose.position.y
        self.postd.pose.position.z = msg.pose.position.z


    def isgriiped(self,msg):
        self.posstring.data=msg.data
        

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
    pos.pose.position.z = 3
    

    # Set your velocity here
    vel = TwistStamped()
    vel.twist.linear.x = 5
    vel.twist.linear.y = 5
    vel.twist.linear.z = 5
    # vel.twist.angular.x = 1
    # vel.twist.angular.y = 1
    # vel.twist.angular.z = 1

    
    # Similarly add other containers 

    # Initialize subscriber 
    rospy.Subscriber("/mavros/state",State, stateMt.stateCb)

    # Similarly initialize other subscribers 
    rospy.Subscriber("/mavros/local_position/pose",PoseStamped, stateMt.statepos)

    rospy.Subscriber("/gripper_check",String, stateMt.isgriiped)



    '''
    NOTE: To set the mode as OFFBOARD in px4, it needs atleast 100 setpoints at rate > 10 hz, so before changing the mode to OFFBOARD, send some dummy setpoints  
    '''
    for i in range(100):
        local_pos_pub.publish(pos)
        rate.sleep()


    # Arming the drone
    while not stateMt.state.armed:
        ofb_ctl.setArm()
        while not stateMt.state.mode=="OFFBOARD":
            local_pos_pub.publish(pos)
            ofb_ctl.offboard_set_mode()
            rate.sleep()
    print("Armed!!")

    # Switching the state to auto mode
    # while not stateMt.state.mode=="OFFBOARD":

    #     ofb_ctl.offboard_set_mode()
    #     rate.sleep()
    print ("OFFBOARD mode activated")
    # while not stateMt.posstring.data=="True":
    #     ofb_ctl.setGrip_on_object()
    #     rate.sleep()

    p1=False
    p2=True
    p3=True
    p4=True
    p5=True
    p6=True
    p7=True
    p8=True
    p9=True
    p10=True
    p11=True
    p12=True
    local_vel_pub.publish(vel)
    

    # Publish the setpoints 
    while not rospy.is_shutdown():
        

        distance_tolerance=0.1
        
        while p1==False :
            pos.pose.position.x = 0
            pos.pose.position.y = 0
            pos.pose.position.z = 3
            local_pos_pub.publish(pos)
            # local_vel_pub.publish(vel)
            rate.sleep()
            
            if  abs(stateMt.postd.pose.position.x)<= distance_tolerance*2 and abs(stateMt.postd.pose.position.z-3)<= distance_tolerance and abs(stateMt.postd.pose.position.y)<= distance_tolerance*2:
                p2=False 
                p1=True
                print( stateMt.postd.pose.position.x)
                print( stateMt.postd.pose.position.y)
                print( stateMt.postd.pose.position.z)
            else:
                p2=True

        while p2==False :
            pos.pose.position.x = 3
            pos.pose.position.y = 0
            pos.pose.position.z = 3 
            local_pos_pub.publish(pos)
            # local_pos_pub.publish(vel)
            rate.sleep()
            if  abs(stateMt.postd.pose.position.x-3)<= distance_tolerance and abs(stateMt.postd.pose.position.z-3)<= distance_tolerance and abs(stateMt.postd.pose.position.y)<= distance_tolerance:
                p3=False
                p2=True
                # ofb_ctl.disArm()
                # rate.sleep()
                
                print( stateMt.postd.pose.position.x)
                print( stateMt.postd.pose.position.y)
                print( stateMt.postd.pose.position.z)
            else:
                p3=True

        while p3==False :
            pos.pose.position.x = 3
            pos.pose.position.y = 0
            pos.pose.position.z = 0.1
            local_pos_pub.publish(pos)
            
            rate.sleep()
            if  abs(stateMt.postd.pose.position.x-3)<= distance_tolerance*0.1 and abs(stateMt.postd.pose.position.y)<= distance_tolerance*0.1 and abs(stateMt.postd.pose.position.z-0.1)<= distance_tolerance:
                while not stateMt.posstring.data:
                    print("Drone is not properly placed !!!")
                    return
                ofb_ctl.land_mode()
                rate.sleep()

                print( stateMt.postd.pose.position.x)
                print( stateMt.postd.pose.position.y)
                print( stateMt.postd.pose.position.z)
                for i in range(40):
                    rate.sleep()
                ofb_ctl.setGrip_on_object()
                for i in range(150):
                    local_pos_pub.publish(pos)
                    rate.sleep()
                while not stateMt.state.armed:
                    local_pos_pub.publish(pos)
                    ofb_ctl.setArm()
                    while not stateMt.state.mode=="OFFBOARD":
                        ofb_ctl.offboard_set_mode()
                     # rate.sleep()
                print("Armed!!")
                while not stateMt.state.mode=="OFFBOARD":
                    ofb_ctl.offboard_set_mode()
                    rate.sleep()
                print ("OFFBOARD mode activated")
                pos.pose.position.x = 3
                pos.pose.position.y = 0
                pos.pose.position.z = 3
                local_pos_pub.publish(pos)
                # for i in range(80):
                #     rate.sleep()
                p5=False
                p3=True
                
            else:
                p5=True
            


        while p5==False :
            
            pos.pose.position.x = 3
            pos.pose.position.y = 0
            pos.pose.position.z = 3
            
            local_pos_pub.publish(pos)
            
            rate.sleep()
            if  abs(stateMt.postd.pose.position.z-3)<= distance_tolerance and abs(stateMt.postd.pose.position.x-3)<= distance_tolerance and abs(stateMt.postd.pose.position.y)<= distance_tolerance:
                p6=False
                # ofb_ctl.looseGrip_on_object()
                p5=True
                print( stateMt.postd.pose.position.x)
                print( stateMt.postd.pose.position.y)
                print( stateMt.postd.pose.position.z)
            else:
                p6=True

            # print(stateMt.isgriiped(msg))
            
        while p6==False :
            pos.pose.position.x = 3
            pos.pose.position.y = 3
            pos.pose.position.z = 3
            
            local_pos_pub.publish(pos)
            rate.sleep()
            if  abs(stateMt.postd.pose.position.z-3)<= distance_tolerance*2 and abs(stateMt.postd.pose.position.x-3)<= distance_tolerance*2 and abs(stateMt.postd.pose.position.y-3)<= distance_tolerance*2:
                p6=True
                p7=False
                # ofb_ctl.looseGrip_on_object()
                print( stateMt.postd.pose.position.x)
                print( stateMt.postd.pose.position.y)
                print( stateMt.postd.pose.position.z)
            else:
                p7=True

        while p7==False :
            pos.pose.position.x = 3
            pos.pose.position.y = 3
            pos.pose.position.z = 0
           
            local_pos_pub.publish(pos)
            # print("I reached finally here ;)")
            if  abs(stateMt.postd.pose.position.z)<= distance_tolerance and abs(stateMt.postd.pose.position.x-3)<= distance_tolerance and abs(stateMt.postd.pose.position.y-3)<= distance_tolerance:
                print(stateMt.postd.pose.position.x)
                print(stateMt.postd.pose.position.y)
                print(stateMt.postd.pose.position.z)
                ofb_ctl.looseGrip_on_object()
                # ofb_ctl.land_mode()
                # for i in range(80):
                #     rate.sleep()
                # rate.sleep()
                
                
            
                for i in range(100):
                    local_pos_pub.publish(pos)
                    rate.sleep()
                while not stateMt.state.armed:
                    ofb_ctl.setArm()
                    while not stateMt.state.mode=="OFFBOARD":
                        ofb_ctl.offboard_set_mode()
                     # rate.sleep()
                print("Armed!!")
                while not stateMt.state.mode=="OFFBOARD":
                    ofb_ctl.offboard_set_mode()
                    rate.sleep()
                print ("OFFBOARD mode activated")
                pos.pose.position.x = 3
                pos.pose.position.y = 3
                pos.pose.position.z = 3
                local_pos_pub.publish(pos)
                p7=True
                p9=False
                
            else:
                p9=True

        
        if p9==False :
            pos.pose.position.x = 3
            pos.pose.position.y = 3
            pos.pose.position.z = 3
            
            local_pos_pub.publish(pos)
            rate.sleep()
            if  abs(stateMt.postd.pose.position.z-3)<= distance_tolerance and abs(stateMt.postd.pose.position.x-3)<= distance_tolerance and abs(stateMt.postd.pose.position.y-3)<= distance_tolerance:
                p9=True
                p10=False
                print( stateMt.postd.pose.position.x)
                print( stateMt.postd.pose.position.y)
                print( stateMt.postd.pose.position.z)
            else:
                p10=True

        if p10==False :
            pos.pose.position.x = 0
            pos.pose.position.y = 0
            pos.pose.position.z = 3
            
            local_pos_pub.publish(pos)
            
            rate.sleep()
            if  abs(stateMt.postd.pose.position.z-3)<= distance_tolerance and abs(stateMt.postd.pose.position.x)<= distance_tolerance and abs(stateMt.postd.pose.position.y)<= distance_tolerance:
                p10=True
                p11=False
                print( stateMt.postd.pose.position.x)
                print( stateMt.postd.pose.position.y)
                print( stateMt.postd.pose.position.z)
            else:
                p10=False

        

        if p11==False :
            pos.pose.position.x = 0
            pos.pose.position.y = 0
            pos.pose.position.z = 0
            
            local_pos_pub.publish(pos)
            local_vel_pub.publish(vel)
            rate.sleep()
            if  abs(stateMt.postd.pose.position.z)<= distance_tolerance and abs(stateMt.postd.pose.position.x)<= distance_tolerance and abs(stateMt.postd.pose.position.y)<= distance_tolerance:
                p11=True
                p12=False
                print( stateMt.postd.pose.position.x)
                print( stateMt.postd.pose.position.y)
                print( stateMt.postd.pose.position.z)
            else:
                p11=False

        if p12==False:
            return 

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass