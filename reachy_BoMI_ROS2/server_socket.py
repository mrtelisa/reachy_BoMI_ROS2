#!/usr/bin/env python3

from posixpath import split
import socket
import threading
from xmlrpc.client import Server
import numpy as np
import os
# import settings
from queue import Queue
import subprocess
import time
import rosgraph

# ROS
import rospy
from geometry_msgs.msg import *
from std_msgs.msg import *
from nav_msgs.msg import *

#from my_tiago.msg import Velocities

#GLOBAL VARIABLES
HEADER = 64
PORT = 5051
SERVER = [l for l in ([ip for ip in socket.gethostbyname_ex(socket.gethostname())[2] if not ip.startswith("127.")][:1], [[(s.connect(('8.8.8.8', 53)), s.getsockname()[0], s.close()) for s in [socket.socket(socket.AF_INET, socket.SOCK_DGRAM)]][0][1]]) if l][0][0]
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"


class ServerData:
    """
    This class is used to share variables between functions
    """
    def __init__(self):

        ##TELEOPERATION BASE ############
        #Variables
        self.linear_vel = None
        self.angular_vel = None
        self.x_coordinate  = None
        self.y_coordinate = None
        self.base_state = None
        self.arm_state = None
        self.base_state_pub = None
        self.arm_state_pub = None
        self.send_coordinates = False
        self.map_name = None
        self.vector_angle = None
        self.vector_amplitude = None
        self.send_vector_info = False
        
        #ROS Publisher
        self.linear_vel_pub = None
        self.ang_vel_pub = None
        self.x_coor_pub = None
        self.y_coor_pub = None
        self.map_name_pub = None
        self.amplitude_vector_pub = None
        self.angle_vector_pub = None

        ###################################

#Instanciate an object of the class
server_data = ServerData()


## FUNCTIONS

def handle_client(conn, addr):
    global linear_vel
    print(f"[NEW CONNECTION] {addr} connected.")

    connected = True
    while connected:
        msg = conn.recv(1024).decode(FORMAT)
        decode_msg(msg)
        if msg == DISCONNECT_MESSAGE:
            connected = False

        print(f"[{addr}] {msg}")

    conn.close()




def decode_msg(msg):
    """
    Function used to decode the msg sent by the socket client
    :param
        msg: string received by client
    """
    global server_data

    #check for lin_vel in msg
    if 'lin_vel:' in msg:
        position = msg.find('lin_vel:')
        float_pos = position + len("lin_vel:")
        if msg[float_pos] == '-':
            lin_vel =msg[float_pos:float_pos + 4]
        else:
            lin_vel = msg[float_pos: float_pos + 3]
            flag_negative = False
        server_data.linear_vel = float(lin_vel)

    
    #check for ang_vel in msg

    if 'ang_vel:' in msg:
        position = msg.find('ang_vel:')
        float_pos = position + len("ang_vel:")
        if msg[float_pos] == '-':
            ang_vel = msg[float_pos:float_pos + 4]
        else:
            ang_vel = msg[float_pos: float_pos + 3]

        server_data.angular_vel = float(ang_vel)

    #Extract amplitude and angle of the 2D vector
    if "angle" in msg:
        splitted_msg = msg.split(" ")
        angle = splitted_msg[1]
        server_data.vector_angle = int(angle)
        amplitude = splitted_msg[3]
        server_data.vector_amplitude = int(amplitude)
        server_data.send_vector_info = True

    #check for target position for odom GUI
    if 'x:' in msg:
        position = msg.find('x:')
        int_pos = position + len('x:')
        if msg[int_pos] == '-':
            x_coo = msg[int_pos:int_pos + 2]
        else:
            x_coo = msg[int_pos]

        server_data.x_coordinate = float(x_coo)

    if 'y:' in msg:
        position = msg.find('y:')
        int_pos = position + len('y:')
        if msg[int_pos] == '-':
            y_coo = msg[int_pos:int_pos + 2]
        else:
            y_coo = msg[int_pos]

        server_data.y_coordinate = float(y_coo)
        server_data.send_coordinates = True

    #check for start simple office simulation

    #LEGEND
    #map_name = 1.0 simple_office
    #map_name = 2.0 simple_office_with_people

    if 'familiarization' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./familiarization.sh')
        print("[STARTING ...] Familiarization")
        server_data.map_name = 1.0
        server_data.map_name_pub.publish(msg)


    elif'initial_test' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./initial_test.sh')
        print("[STARTING ...] Initial Test")
        server_data.map_name = 2.0
        server_data.map_name_pub.publish(msg)


    elif 'train_1' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./train_1.sh')
        print("[STARTING ...] Train")
        server_data.map_name = 3.0
        server_data.map_name_pub.publish(msg)

    elif 'train_2' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./train_2.sh')
        print("[STARTING ...] Train")
        server_data.map_name = 4.0
        server_data.map_name_pub.publish(msg)

    elif 'train_3' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./train_3.sh')
        print("[STARTING ...] Train")
        server_data.map_name = 5.0
        server_data.map_name_pub.publish(msg)

    elif 'train_4' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./train_4.sh')
        print("[STARTING ...] Train")
        server_data.map_name = 6.0
        server_data.map_name_pub.publish(msg)

    elif'mid_test' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./mid_test.sh')
        print("[STARTING ...] Mid Test")
        server_data.map_name = 7.0
        server_data.map_name_pub.publish(msg)

    elif 'train_5' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./train_5.sh')
        print("[STARTING ...] Train")
        server_data.map_name = 8.0
        server_data.map_name_pub.publish(msg)
    elif 'train_6' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./train_6.sh')
        print("[STARTING ...] Train")
        server_data.map_name = 9.0
        server_data.map_name_pub.publish(msg)

    elif 'train_7' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./train_7.sh')
        print("[STARTING ...] Train")
        server_data.map_name = 10.0
        server_data.map_name_pub.publish(msg)

    elif 'train_8' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./train_8.sh')
        print("[STARTING ...] Train")
        server_data.map_name = 11.0
        server_data.map_name_pub.publish(msg)
        
    elif'final_test' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        os.system('./final_test.sh')
        print("[STARTING ...] Final Test")
        server_data.map_name = 12.0
        server_data.map_name_pub.publish(msg)
   

    #elif 'simple_office_with_people' == msg:
        # ---EXECUTE THE CORRESPONDING BASH FILE ---#
        #os.system('./simple_office_with_people.sh')
        #print("[STARTING ...] Simple Office With People")
        #server_data.map_name = 2.0
        #server_data.map_name_pub.publish(msg)

    #elif 'real_tiago' == msg:
        #os.system('./real_tiago.sh')
        #server_data.map_name = 3.0
        #server_data.map_name_pub.publish(server_data.map_name)


    # ---CHECK AND UPDATE THE STATE OF THE BASE ---#
    # 'nine regions' --> +1.0
    # 'odom' --> +0.0

    if "nine region" in msg:
        server_data.base_state = 1.0
        server_data.arm_state = -1.0
        # -- Update arm and base states -- #
        server_data.base_state_pub.publish(server_data.base_state)
        server_data.map_name_pub.publish(server_data.map_name)
        server_data.arm_state_pub.publish(server_data.arm_state)

    if "odom" in msg:
        server_data.base_state = 0.0
        server_data.arm_state = -1.0
        # -- Update arm and base states -- #
        server_data.base_state_pub.publish(server_data.base_state)
        server_data.map_name_pub.publish(server_data.map_name)
        server_data.arm_state_pub.publish(server_data.arm_state)

    if "2D vector" in msg:
        server_data.base_state = -1.0
        server_data.arm_state = 1.0
        # -- Update arm,base states and map -- #
        server_data.arm_state_pub.publish(server_data.arm_state)
        server_data.base_state_pub.publish(server_data.base_state)
        server_data.map_name_pub.publish(server_data.map_name)


    if "1D vector" in msg:
        server_data.base_state = -1.0
        server_data.arm_state = 0.0
        # -- Update arm,base states and map -- #
        server_data.arm_state_pub.publish(server_data.arm_state)
        server_data.base_state_pub.publish(server_data.base_state)
        server_data.map_name_pub.publish(server_data.map_name)


def init_publishers():
    global server_data
    roscore = subprocess.Popen('roscore')
    time.sleep(1)
    rospy.init_node("server_socket", anonymous=True)
    print("Server Node STARTED")

    # -- Declare all ROS publishers -- #
    server_data.linear_vel_pub = rospy.Publisher("server_socket/linear_vel",Float32,queue_size=10)
    server_data.ang_vel_pub = rospy.Publisher("server_socket/angular_vel",Float32,queue_size=10)
    #velocities_publisher = rospy.Publisher("server_socket/velocities",Velocities,queue_size=10)
    server_data.x_coor_pub = rospy.Publisher("server_socket/x_coordinate",Float32,queue_size=10)
    server_data.y_coor_pub = rospy.Publisher("server_socket/y_coordinate",Float32,queue_size=10)
    server_data.map_name_pub = rospy.Publisher("server_socket/map_name",Float32,queue_size=10)
    server_data.base_state_pub = rospy.Publisher("server_socket/base_state",Float32,queue_size=10)
    server_data.arm_state_pub = rospy.Publisher("server_socket/arm_state",Float32,queue_size=10)
    server_data.amplitude_vector_pub = rospy.Publisher("server_socket/vector_amplitude", Int32, queue_size=10)
    server_data.angle_vector_pub = rospy.Publisher("server_socket/vector_angle", Int32, queue_size=10)



    rate = rospy.Rate(20)
    server_data.linear_vel = 0.0
    server_data.angular_vel = 0.0

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():

        # -- Nine Region GUI -- #
        if server_data.base_state == 1.0:

            try:
                # msg = Velocities()
                # msg.linear_velocity = server_data.linear_vel
                # msg.angular_velocity = server_data.angular_vel
                # velocities_publisher.publish(msg)
                server_data.linear_vel_pub.publish(server_data.linear_vel)
                server_data.ang_vel_pub.publish(server_data.angular_vel)
                server_data.base_state_pub.publish(server_data.base_state)
                server_data.arm_state_pub.publish(server_data.arm_state)
                #server_data.map_name_pub.publish(server_data.map_name)

                rate.sleep()
                #print("Velocità pubblicate")
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
        
        elif server_data.base_state == 0.0 and server_data.send_coordinates:
            try:
                server_data.x_coor_pub.publish(server_data.x_coordinate)
                server_data.y_coor_pub.publish(server_data.y_coordinate)
                server_data.base_state_pub.publish(server_data.base_state)
                server_data.arm_state_pub.publish(server_data.arm_state)
                server_data.send_coordinates = False
               # server_data.map_name_pub.publish(server_data.map_name)
                print("Coordinate Pubblicate")
            except rospy.ROSInterruptException:
                rospy.logerr("ROS Interrupt Exception! Just ignore the exception!")
        

        #For 1D vector
        elif server_data.arm_state == 0.0 and server_data.send_vector_info:
            server_data.arm_state_pub.publish(server_data.arm_state)
            server_data.base_state_pub.publish(server_data.base_state)
            server_data.amplitude_vector_pub.publish(server_data.vector_amplitude)
            server_data.angle_vector_pub.publish(server_data.vector_angle)
            #Restore to False to the next turn
            server_data.send_vector_info = False

        #For 2D vector
        elif server_data.arm_state == 1.0 and server_data.send_vector_info:
            server_data.arm_state_pub.publish(server_data.arm_state)
            server_data.base_state_pub.publish(server_data.base_state)
            server_data.amplitude_vector_pub.publish(server_data.vector_amplitude)
            server_data.angle_vector_pub.publish(server_data.vector_angle)
            #Restore to False to the next turn
            server_data.send_vector_info = False



if __name__ == '__main__':
    """
    Entry point of the script
    In particular it initialize the file used for manage the variables.
    It starts the server putting it in listening mode.
    Once the client is connected to the server it starts a thread for receive the messages from the client
    """
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(ADDR)
    print("[STARTING] server is starting...")
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")
    # while True:
    conn, addr = server.accept()
    thread = threading.Thread(target=handle_client, args=(conn, addr))
    thread.start()
    # thread.join()
    print(f"[ACTIVE CONNECTIONS] {threading.activeCount() - 1}")
    init_publishers()







