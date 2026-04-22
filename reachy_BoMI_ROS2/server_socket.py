#!/usr/bin/env python3

import socket
import threading
import numpy as np
import os
import subprocess
import time

# ROS 2 LIBRARIES
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Float32, Int32

current_dir = os.path.dirname(os.path.abspath(__file__))

# SOCKET CONFIGURATION
HEADER = 64
PORT = 5051
# Dynamic IP
try:
    SERVER = socket.gethostbyname(socket.gethostname())
except:
    SERVER = '127.0.0.1' 
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"

class ServerSocketNode(Node):
    """
    ROS2 node that handles Publishers and recieved data from Socket
    """
    def __init__(self):
        super().__init__('server_socket_node')

        # --- State variables ---
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.x_coordinate = 0.0
        self.y_coordinate = 0.0
        self.base_state = -1.0
        self.map_name = None
        self.vector_angle = 0
        self.vector_amplitude = 0
        
        self.send_coordinates = False
        self.send_vector_info = False

        # --- PUBLISHERS ---
        self.linear_vel_pub = self.create_publisher(Float32, "server_socket/linear_vel", 10)
        self.ang_vel_pub = self.create_publisher(Float32, "server_socket/angular_vel", 10)
        self.x_coor_pub = self.create_publisher(Float32, "server_socket/x_coordinate", 10)
        self.y_coor_pub = self.create_publisher(Float32, "server_socket/y_coordinate", 10)
        self.map_name_pub = self.create_publisher(Float32, "server_socket/map_name", 10)
        self.base_state_pub = self.create_publisher(Float32, "server_socket/base_state", 10)
        self.amplitude_vector_pub = self.create_publisher(Int32, "server_socket/vector_amplitude", 10)
        self.angle_vector_pub = self.create_publisher(Int32, "server_socket/vector_angle", 10)

        # --- TIMER LOOP (20Hz) ---
        self.timer = self.create_timer(0.05, self.publish_loop)
        self.get_logger().info(f"ROS 2 Server Node avviato su IP: {SERVER}")


    def publish_loop(self):
        # Continously publishing base state
        self.base_state_pub.publish(Float32(data=float(self.base_state)))

        # Nine Region GUI
        if self.base_state == 1.0:
            self.linear_vel_pub.publish(Float32(data=self.linear_vel))
            self.ang_vel_pub.publish(Float32(data=self.angular_vel))
            self.get_logger().info("Velocities published")

        # Odom GUI (Coordinates)
        elif self.base_state == 0.0 and self.send_coordinates:
            self.x_coor_pub.publish(Float32(data=self.x_coordinate))
            self.y_coor_pub.publish(Float32(data=self.y_coordinate))
            self.send_coordinates = False
            self.get_logger().info("Coordinates published")


    def decode_msg(self, msg):
        """
        Function used to decode the msg sent by the socket client
        :param
            msg: string received by client
        """
        try:
            if 'lin_vel:' in msg:
                # ex: "lin_vel:0.5"
                val = msg.split('lin_vel:')[1][:4]
                self.linear_vel = float(val)

            if 'ang_vel:' in msg:
                val = msg.split('ang_vel:')[1][:4]
                self.angular_vel = float(val)

            if "angle" in msg:
            # Extract amplitude and angle of the 2D vector
                parts = msg.split(" ")
                self.vector_angle = int(parts[1])
                self.vector_amplitude = int(parts[3])
                self.send_vector_info = True

            # Check for target position for odom GUI
            if 'x:' in msg:
                self.x_coordinate = float(msg.split('x:')[1].split()[0])
            if 'y:' in msg:
                self.y_coordinate = float(msg.split('y:')[1].split()[0])
                self.send_coordinates = True

            # Mapping scenarios (Bash scripts)
            scenarios = {
                'familiarization': (1.0, os.path.join(current_dir, 'sh_files/familiarization.sh')),
                'initial_test': (2.0, os.path.join(current_dir, 'sh_files/initial_test.sh')),
                'train_1': (3.0, os.path.join(current_dir, 'sh_files/sh_files/train_1.sh')),
                'train_2': (4.0, os.path.join(current_dir, 'sh_files/sh_files/train_2.sh')),
                'train_3': (5.0, os.path.join(current_dir, 'sh_files/sh_files/train_3.sh')),
                'train_4': (6.0, os.path.join(current_dir, 'sh_files/sh_files/train_4.sh')),
                'mid_test': (7.0, os.path.join(current_dir, 'sh_files/sh_files/mid_test.sh')),
                'train_5': (8.0, os.path.join(current_dir, 'sh_files/sh_files/train_5.sh')),
                'train_6': (9.0, os.path.join(current_dir, 'sh_files/sh_files/train_6.sh')),
                'train_7': (10.0, os.path.join(current_dir, 'sh_files/sh_files/train_7.sh')),
                'train_8': (11.0, os.path.join(current_dir, 'sh_files/sh_files/train_8.sh')),
                'final_test': (12.0, os.path.join(current_dir, 'sh_files/sh_files/final_test.sh'))
            }

            for key, (val, script) in scenarios.items():
                if key == msg:
                    os.system(script)
                    self.map_name = val
                    self.map_name_pub.publish(Float32(data=self.map_name))
                    self.get_logger().info(f"Starting scenario {key}")

            # Check and update the state of the base
            if "nine region" in msg:
                self.base_state = 1.0
            elif "odom" in msg:
                self.base_state = 0.0

        except Exception as e:
            self.get_logger().error(f"Errore decodifica: {e}")


# --- SOCKET LOGIC ---
def handle_client(conn, addr, node):
    print(f"[NEW CONNECTION] {addr} connected.")
    connected = True
    while connected:
        try:
            msg = conn.recv(1024).decode(FORMAT)
            if not msg: break
            
            node.decode_msg(msg)
            
            if msg == DISCONNECT_MESSAGE:
                connected = False
            print(f"[{addr}] {msg}")
        except:
            break
    conn.close()


def main():
    # ROS2 initialization
    rclpy.init()
    node = ServerSocketNode()

    # Starting Socket Server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(ADDR)
    server_socket.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")

    # Starting ROS2 in a separate thread 
    ros_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    ros_thread.start()

    try:
        while True:
            conn, addr = server_socket.accept()
            client_thread = threading.Thread(target=handle_client, args=(conn, addr, node))
            client_thread.start()
            print(f"[ACTIVE CONNECTIONS] {threading.activeCount() - 2}")
    except KeyboardInterrupt:
        print("[SHUTTING DOWN] Server in chiusura...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()