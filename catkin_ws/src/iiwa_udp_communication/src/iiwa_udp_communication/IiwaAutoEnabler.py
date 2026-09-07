#!/usr/bin/env python

import rospy
from iiwa_udp_communication import IiwaMessengerROS


class IiwaAutoEnabler:
    def __init__(self):
        self.messenger = IiwaMessengerROS()

    def __del__(self):
        self.disable()

    def disable(self):
        self.messenger.send_message("App_Enable;false")

    def enable(self):
        # enable robot
        self.messenger.send_message("App_Enable;true")

        # start application by sending a rising edge
        # (false->true) for App_Start
        self.messenger.send_message("App_Start;false")
        self.messenger.send_message("App_Start;true")

        #TODO: detect if program running correctly

        # Send enable message. Program is paused if the robot does not
        # receive an App_Enable message for 100ms
        rate = rospy.Rate(25)
        try:
            while not rospy.is_shutdown():
                self.messenger.send_message("App_Enable;true")
                rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            # shutdown requested
            pass

        # ros shutdown so stop program
        self.messenger.send_message("App_Enable;false")
