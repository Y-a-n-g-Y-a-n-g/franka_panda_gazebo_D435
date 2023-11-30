#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import tf.transformations
import numpy as np
from geometry_msgs.msg import PoseStamped
from franka_msgs.msg import FrankaState
import socket
import threading
import math
marker_pose = PoseStamped()
pose_pub = None
position_limits = [[-0.6, 0.6], [-0.6, 0.6], [0.05, 0.9]]
SafeWorkspace=[[-0.2, 0.7], [-0.6, 0.6], [0.05, 0.7]]
startpose=[0.3,0,0.5,1,0,0,0]
nowpose=None
stepxyz=0.005
stepangle=1
def rotate_quaternion(quaternion, axis, angle):
    # 将旋转轴向量标准化
    magnitude = math.sqrt(axis[0] ** 2 + axis[1] ** 2 + axis[2] ** 2)
    axis = [axis[0] / magnitude, axis[1] / magnitude, axis[2] / magnitude]

    # 将旋转角度转换为弧度
    angle = math.radians(angle)

    # 计算旋转的sin和cos值
    sin_half_angle = math.sin(angle / 2)
    cos_half_angle = math.cos(angle / 2)

    # 计算旋转后的四元数的每个分量
    rotated_quaternion = [
        cos_half_angle * quaternion[0] + sin_half_angle * (axis[1] * quaternion[2] - axis[2] * quaternion[1]),
        cos_half_angle * quaternion[1] + sin_half_angle * (axis[2] * quaternion[0] - axis[0] * quaternion[2]),
        cos_half_angle * quaternion[2] + sin_half_angle * (axis[0] * quaternion[1] - axis[1] * quaternion[0]),
        cos_half_angle * quaternion[3] - sin_half_angle * (axis[0] * quaternion[0] + axis[1] * quaternion[1] + axis[2] * quaternion[2])]
    return rotated_quaternion

def publisher_callback(msg, link_name):
    marker_pose.header.frame_id = link_name
    marker_pose.header.stamp = rospy.Time(0)
    pose_pub.publish(marker_pose)
def wait_for_initial_pose():
    global nowpose
    msg = rospy.wait_for_message("franka_state_controller/franka_states",FrankaState) 
    initial_quaternion = tf.transformations.quaternion_from_matrix( np.transpose(np.reshape(msg.O_T_EE,(4, 4))))
    initial_quaternion = initial_quaternion / np.linalg.norm(initial_quaternion)
    marker_pose.pose.orientation.x = initial_quaternion[0]
    marker_pose.pose.orientation.y = initial_quaternion[1]
    marker_pose.pose.orientation.z = initial_quaternion[2]
    marker_pose.pose.orientation.w = initial_quaternion[3]
    marker_pose.pose.position.x = msg.O_T_EE[12]
    marker_pose.pose.position.y = msg.O_T_EE[13]
    marker_pose.pose.position.z = msg.O_T_EE[14]
    nowpose=[msg.O_T_EE[12],msg.O_T_EE[13],msg.O_T_EE[14],initial_quaternion[0],initial_quaternion[1],initial_quaternion[2],initial_quaternion[3]]
def setpose(pose):
    if pose:
        marker_pose.pose.position.x = pose[0]
        marker_pose.pose.position.y = pose[1]
        marker_pose.pose.position.z = pose[2]
        marker_pose.pose.orientation.x = pose[3]
        marker_pose.pose.orientation.y = pose[4]
        marker_pose.pose.orientation.z = pose[5]
        marker_pose.pose.orientation.w = pose[6]
def round4(numlist):
    for i in range(len(numlist)):
        numlist[i]=round(numlist[i],5)
    return numlist
def handle_client(client_socket):
    global  nowpose
    while True:
        data = client_socket.recv(1024)
        if not data:  
            break
        rospy.loginfo(f"Received data: {data.decode()}")
        
        msg = data.decode()
        if nowpose !=None:
            #x
            if msg=='TraX+':
                nowpose[0] =nowpose[0]+stepxyz if nowpose[0]+stepxyz<SafeWorkspace[0][1] else SafeWorkspace[0][1]
                nowpose[0]=round(nowpose[0],5)
            if msg=='TraX-':
                nowpose[0] =nowpose[0]-stepxyz if nowpose[0]-stepxyz >SafeWorkspace[0][0] else SafeWorkspace[0][0]
                nowpose[0]=round(nowpose[0],5)
            #y
            if msg=='TraY+':
                nowpose[1]=nowpose[1]+stepxyz if nowpose[1]+stepxyz<SafeWorkspace[1][1] else SafeWorkspace[1][1]
                nowpose[1]=round(nowpose[1],5)
            if msg=='TraY-':
                nowpose[1]=nowpose[1]-stepxyz if nowpose[1]-stepxyz>SafeWorkspace[1][0] else SafeWorkspace[1][0]
                nowpose[1]=round(nowpose[1],5)
            #z
            if msg=='TraZ+':
                nowpose[2]=nowpose[2]+stepxyz if nowpose[2]+stepxyz<SafeWorkspace[2][1] else SafeWorkspace[2][1]
                nowpose[2]=round(nowpose[2],5)
            if msg=='TraZ-':
                nowpose[2]=nowpose[2]-stepxyz if nowpose[2]-stepxyz>SafeWorkspace[2][0] else SafeWorkspace[2][0]
                nowpose[2]=round(nowpose[2],5)
            #rotatex
            if msg=='RotX+':
                nowpose[3:7]=rotate_quaternion(nowpose[3:7],[1,0,0],stepangle)
            if msg=='RotX-':
                nowpose[3:7]=rotate_quaternion(nowpose[3:7],[1,0,0],-stepangle)
            #rotatey
            if msg=='RotY+':
                nowpose[3:7]=rotate_quaternion(nowpose[3:7],[0,1,0],stepangle)
            if msg=='RotY-':
                nowpose[3:7]=rotate_quaternion(nowpose[3:7],[0,1,0],-stepangle)
            #rotatez
            if msg=='RotZ+':
                nowpose[3:7]=rotate_quaternion(nowpose[3:7],[0,0,1],stepangle)
            if msg=='RotZ-':
                nowpose[3:7]=rotate_quaternion(nowpose[3:7],[0,0,1],-stepangle)
            #MoveToStart
            if msg=='MoveToStart':
                nowpose=startpose.copy()
        setpose(nowpose)
        print(nowpose)
        print(round4(nowpose))
        
    client_socket.close()
def tcp_server(port):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(('localhost', port))
    server_socket.listen(5)
    rospy.loginfo(f"Server listening on port {port}")
    while True:
        client_socket, addr = server_socket.accept()
        rospy.loginfo(f"Accepted connection from {addr}")
        client_thread = threading.Thread(target=handle_client, args=(client_socket,))
        client_thread.start()
        
if __name__ == "__main__":
    rospy.loginfo(f"interactive_marker node start")
    
    t = threading.Thread(target=tcp_server, args=(8080,))
    t.start()
    
    rospy.init_node("equilibrium_pose_node")
    listener = tf.TransformListener()
    link_name = rospy.get_param("~link_name")
    wait_for_initial_pose()
    pose_pub = rospy.Publisher("equilibrium_pose", PoseStamped, queue_size=10)
    rospy.Timer(rospy.Duration(0.005),lambda msg: publisher_callback(msg, link_name))
    rospy.spin()
