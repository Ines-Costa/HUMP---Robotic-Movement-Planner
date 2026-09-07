#!/usr/bin/env python

import socket
import rospy
from iiwa_udp_communication import IiwaMessenger


class IiwaMessengerROS(IiwaMessenger):
    def __init__(self):
        robot_ip = rospy.get_param("~robot_ip", "172.31.1.147")
        robot_socket = rospy.get_param("~robot_socket", 30300)
        own_socket = rospy.get_param("~own_socket", 30333)
        timeout = rospy.get_param("~timeout", 3.0)

        super(IiwaMessengerROS, self).__init__(robot_ip=robot_ip,
                                               robot_socket=robot_socket,
                                               own_socket=own_socket,
                                               timeout=timeout)

    def get_state(self):
        try:
            return super(IiwaMessengerROS, self).get_state()
        except socket.timeout as e:
            msg = ("Socket timed out. Is the robot connected, in auto mode, "
                   "and set up to receive udp messages on {}:{}?".format(
                       self.robot_ip, self.robot_socket))
            rospy.logerr(msg)
            raise e
