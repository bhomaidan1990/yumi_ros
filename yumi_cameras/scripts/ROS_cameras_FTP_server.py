#!/usr/bin/env python

import os
from pyftpdlib.authorizers import DummyAuthorizer
from pyftpdlib.handlers import FTPHandler
from pyftpdlib.servers import FTPServer
import numpy as np
import cv2
import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image



authorizer = None
handler = None
server = None

left_cam_pub = None
right_cam_pub = None
bridge = CvBridge()

pixels_encoding = "mono8"


def publish_image_left_cam(img):
    global left_cam_pub, bridge
    try:
        left_cam_pub.publish(bridge.cv2_to_imgmsg(img, pixels_encoding))
    except CvBridgeError as e:
        rospy.loginfo(e)


def publish_image_right_cam(img):
    global right_cam_pub, bridge
    try:
        right_cam_pub.publish(bridge.cv2_to_imgmsg(img, pixels_encoding))
    except CvBridgeError as e:
        rospy.loginfo(e)


class CamerasFTPHandler(FTPHandler):
    def on_file_received(self, file):
        # print("Hey! File received. Name:")
        # print(file)
        img = cv2.imread(file, 0)
        if (img is not None):
            if("img_left" in file):
                rospy.loginfo("Left camera image")
                publish_image_left_cam(img)
        
            elif ("img_right" in file):
                rospy.loginfo("Right camera image")
                publish_image_right_cam(img)



    def on_incomplete_file_received(self, file):
        rospy.loginfo("Hey! Incomplete file received. Name:" + file)
        os.remove(file)



def start_FTP_server():
    global authorizer, handler, server

    # Instantiate a dummy authorizer for managing 'virtual' users
    authorizer = DummyAuthorizer()

    # Define two users with full r/w permissions, one for each camera
    user_home_folder = os.getenv("HOME")
    authorizer.add_user('right_cam', 'yumiPC', user_home_folder, perm='elradfmwM')
    authorizer.add_user('left_cam', 'yumiPC', user_home_folder, perm='elradfmwM')

    # Instantiate FTP handler class
    handler = CamerasFTPHandler
    handler.authorizer = authorizer

    # Define a customized banner (string returned when client connects)
    handler.banner = "pyftpdlib based ftpd ready."
    handler.permit_privileged_ports = True

    # Instantiate FTP server class and listen on 0.0.0.0:2121
    address = ('192.168.125.50', 2121)
    server = FTPServer(address, handler)

    # set a limit for connections
    server.max_cons = 512
    server.max_cons_per_ip = 0      # 0 == no limit

    rospy.loginfo("FTP server ready, listening on Port 21")
    # start ftp server
    server.serve_forever()



def close_FTP_server():
    global server
    server.close_all()



def main():
    global left_cam_pub, right_cam_pub, server
    rospy.init_node("YumiCamerasNode", anonymous=False)
    rospy.on_shutdown(close_FTP_server)

    left_cam_pub = rospy.Publisher("/yumi/left_cam_image", Image, queue_size=1)
    right_cam_pub = rospy.Publisher("/yumi/right_cam_image", Image, queue_size=1)

    rospy.loginfo("Starting FTP server")

    start_FTP_server()
    


if __name__ == '__main__':
    main()