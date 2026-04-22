#! /usr/bin/env python3

#ROS LIBRARIES

from http import server
from re import T
from wsgiref.simple_server import server_version
import rospy
from geometry_msgs.msg import Twist,Point
from nav_msgs.msg import *
from move_base_msgs.msg import *
from std_msgs.msg import *

#STD PYTHON LIBRARIES
import actionlib
import math
import time
import os

#GLOBAL VARIABLES

tiago_position = Point()

class ServerData:
    """
    This class is used to share variables between functions
    """
    def __init__(self):
        
        #Variables
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.x_coordinate  = 0.0
        self.x_coordinate_arrived = False
        self.y_coordinate_arrived = False
        self.y_coordinate = 0.0
        self.base_state = -1.0
        self.arm_state = -1.0
        self.send_coordinates = False
        self.map_name = None
        self.already_acquired_map_name = False
        #self.vel_publisher = None
        #self.coor_publisher = None
        
        #ROS Subscriber
        self.linear_vel_sub = None
        self.ang_vel_sub = None
        self.x_coor_sub = None
        self.y_coor_sub = None
        self.map_name_sub = None
        self.base_state_sub = None
        self.arm_state_sub = None

        ## COMPUTE THE MAP REGION
        self.x_min = None
        self.x_max = None
        self.y_min = None
        self.y_max = None

#Instanciate an object of the class
server_data = ServerData()




#FUNCTIONS


def odom_clbk(odom_msg):
    """
    Simple odom callback that update TIAGo position into a global variable
    :param 
        odom_msg: the odom msg from subcriber
    """
    global tiago_position
    tiago_position = odom_msg.pose.pose.position
    #print("TIAGo position: x: " + str(tiago_position.x) + " y: " + str(tiago_position.y))

def euler_dist(point1,point2):
    """
    Simple function that computes the euler distance between two 2D points
    :params
        point1: first of the two points
        point2: second of the two points
    """
    dist = math.sqrt((point2.x - point1.x)**2 + (point2.y - point1.y)**2)
    return dist

def check_valid_coordinate(x_coor,y_coor):
    """
    Simple function that computes if the selected coordinates are valid or not
    Of course they depend on TIAGo position, since taret = TIAGo_pos + selected target
    :params
        x_coor: the x coordinate of the target
        y_coor: the y coordinate of the target
    :return
        flag [bool]: True if the coordinates are valid, False otherwise
    """
    global server_data
    if (x_coor  <= server_data.x_max) and (x_coor >= server_data.x_min):
        if (y_coor  <= server_data.y_max) and (y_coor >= server_data.y_min):
            flag = True
        else:
            flag = False
    else:
        flag = False
    return flag

def recompute_target_coordinates(x_coor,y_coor):
    """
    This function recompute the coordinates of the target, depending on the value of the coordinates
    In particular it found what coordinate is out of the map and assign it the a boundary value
    :params
        x_coor: the x coordinate of the target
        y_coor: the y coordinate of the target
    :return
        x_target: the recomputed x coordinate of the target
        y_target: the recomputed y coordinate of the target
    """
    global server_data

    # -- For x component -- #
    if x_coor < server_data.x_min:
        x_coor = server_data.x_min + .5
    elif x_coor  > server_data.x_max:
        x_coor = server_data.x_max - .5
        
    # -- For y component -- #
    if y_coor < server_data.y_min:
        y_coor = server_data.y_min + .5
    elif y_coor  > server_data.y_max:
        y_coor = server_data.y_max - .5

    return x_coor,y_coor

def map_name_clbk(msg):
    """
    Once the name of the map is received, this function stores the values of the map
    """
    global server_data
    server_data.map_name = msg.data
    
    #extract the name of the map update the min and max values

    if server_data.map_name == 1.0 and  server_data.already_acquired_map_name == False:
        server_data.x_min = -4.35
        server_data.x_max = 4.35
        server_data.y_min = -12.36
        server_data.y_max = 1.32
        server_data.already_acquired_map_name = True
        print("Map Size Acquired")
    
    elif server_data.map_name == 2.0 and  server_data.already_acquired_map_name == False:
        server_data.x_min = -4.25
        server_data.x_max = 4.25
        server_data.y_min = -4.2
        server_data.y_max = 9.3
        server_data.already_acquired_map_name = True
        print("Map Size Acuired")

    #Real TIAGo
    elif server_data.map_name == 3.0 and  server_data.already_acquired_map_name == False:
        #default 500 x 500
        server_data.x_min = -500
        server_data.x_max = 500
        server_data.y_min = -500
        server_data.y_max = 500
        server_data.already_acquired_map_name = True



def base_state_clbk(msg):
    """
    base state clbk that store the value into a global variable
    """
    global server_data
    server_data.base_state = msg.data
    #print("Base State : " + str(msg.data))


def arm_state_clbk(msg):
    """
    arm state clbk that store the value into a global variable
    """
    global server_data
    server_data.arm_state = msg.data



def linear_vel_clbk(msg):
    """
    linear velocity clbk that store the value into a global variable
    """
    global server_data
    server_data.linear_vel = msg.data


def angular_vel_clbk(msg):
    """
    angular velocity clbk that store the value into a global variable
    """
    global server_data
    server_data.angular_vel = msg.data

    
def x_coordinate_clbk(msg):
    """
    x coordinate clbk that store the value into a global variable
    """
    global server_data
    server_data.x_coordinate = msg.data
    print("X coordinate arrived: " + str(msg.data))
    server_data.x_coordinate_arrived = True


def y_coordinate_clbk(msg):
    """
    y coordinate clbk that store the value into a global variable
    """
    global server_data
    server_data.y_coordinate = msg.data
    print("Y coordinate: " + str(msg.data))
    server_data.y_coordinate_arrived = True


def move_tiago():
    """
    Function used to move TIAGo
    If FSM state is to teleoperate the base with 'nine regions gui':
        it publishes on the topic /mobile_base_controller/cmd_vel the values taken from the lin and ang vel topic
    If FSM state is to teleoperate the base with ' odom gui':  
        Once the values (x_coor and y_coor) from the server are modified, it publish on the topic /move_base_simple/goal 
        a new target to be reached by TIAGo
    """
    global tiago_position, server_data

    # Starts a new node
    print("Cmd Vel publisher node [STARTED]")
    rospy.init_node('cmd_vel_publisher', anonymous=True)
    
    #Declare a publisher on the topic /mobile_base_controller/cmd_vel
    cmd_vel_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)

    #Declare a subscriber on /mobile_base_controller/odom
    odom_sub = rospy.Subscriber('/mobile_base_controller/odom',Odometry,odom_clbk)
    
    #Declare a client on the topic /move_base_simple/goal 
    move_base_client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

    #Declare a subscriber on the topic /server_socket/map_name
    server_data.map_name_sub = rospy.Subscriber('server_socket/map_name',Float32,map_name_clbk)

    #Declare a subscriber on the topic /server_socket/base_state
    server_data.base_state_sub = rospy.Subscriber('server_socket/base_state',Float32,base_state_clbk)

    #Declare a subscriber on the topic /server_socket/arm_state
    server_data.arm_state_sub = rospy.Subscriber('server_socket/arm_state',Float32,arm_state_clbk)

    #Declare a subscriber on the topic /server_socket/linear_vel
    server_data.linear_vel_sub = rospy.Subscriber('server_socket/linear_vel',Float32,linear_vel_clbk)

    #Declare a subscriber on the topic /server_socket/angular_vel
    server_data.ang_vel_sub = rospy.Subscriber('server_socket/angular_vel',Float32,angular_vel_clbk)

    #Declare a subscriber on the topic /server_socket/x_coordinate
    server_data.x_coor_sub = rospy.Subscriber('server_socket/x_coordinate',Float32,x_coordinate_clbk)

    #Declare a subscriber on the topic /server_socket/y_coordinate
    server_data.y_coor_sub = rospy.Subscriber('server_socket/y_coordinate',Float32,y_coordinate_clbk)


    
    #Declare a twist msg
    vel_msg = Twist()
    
    #Declare a rate of 10 Hz
    rate = rospy.Rate(10) # 10hz

    counter_seq = 0

    
    #Variable to store the information about the first goal
    #Used to understand if it is necessary to cancel the goal
    first_goal = True



    
    #Main loop
    while not rospy.is_shutdown():

        #if the value == 1.0 --> nine region GUI
        #move TIAGo thanks cmd_vel values
        if server_data.base_state == 1.0:

            #if at least one goal has been published
            if first_goal == False:
                #clear all move base goals
                move_base_client.cancel_all_goals()
            
            vel_msg.linear.x = server_data.linear_vel
            vel_msg.angular.z = server_data.angular_vel

            cmd_vel_publisher.publish(vel_msg)
            print("I have published Lin Vel: " + str(server_data.linear_vel) + " Angular Vel: " + str(server_data.angular_vel))

        #if odom GUI is stated
        if server_data.base_state == 0.0:

            #if both coordinates are arrived
            if server_data.x_coordinate_arrived and server_data.y_coordinate_arrived:
                
                #Restore the values for the next turn
                server_data.x_coordinate_arrived = False
                server_data.y_coordinate_arrived = False

                #Declare the target position
                target_position = Point()
                target_position.x = server_data.x_coordinate + tiago_position.x
                target_position.y = server_data.y_coordinate + tiago_position.y

                #check if the target position is valid
                #if yes then publish as goal with MoveBase
                
                flag = check_valid_coordinate(target_position.x,target_position.y)

                if flag == False:
                    # -- If target out of map limit the component that exceed -- #
                    print("Original Target Out Of map, I am recomputing it and send it!!!")
                    target_position.x ,target_position.y = recompute_target_coordinates(target_position.x,target_position.y)
                    print("Recomputed coordinates are X: " + str(target_position.x) + " Y: " + str(target_position.y))

                print("The selected coordinates are valid!")

                #wait the server and send the goal
                move_base_client.wait_for_server()
                print("Waiting Move Base Server...")

                #Declare and fill a move base goal
                goal = move_base_msgs.msg.MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.header.seq = counter_seq
                counter_seq = counter_seq + 1
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose.position.x = target_position.x
                goal.target_pose.pose.position.y = target_position.y
                goal.target_pose.pose.orientation.w = 1 #default orientation


                move_base_client.send_goal(goal)
                first_goal = False
                print("Goal position sended")
                # wait = move_base_client.wait_for_result()
                # if not wait:
                #     rospy.logerr("Action server not available!")
                #     rospy.signal_shutdown("Action server not available!")
                # else:
                #     print("TIAGo is arrived in x: " + str(target_position.x) + " y: " + str(target_position.y))
                


        rate.sleep()


if __name__ == '__main__':
    """
    Entry point of the script
    It call the function move_tiago until an interrupt exception by the user is given (Ctrl + c)
    """
    try:
        move_tiago()
    except rospy.ROSInterruptException: 
        pass