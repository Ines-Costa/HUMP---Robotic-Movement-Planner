#!/usr/bin/env python

# see https://wiki.python.org/moin/UdpCommunication

import socket
import time


class IiwaMessenger(object):
    __slots__ = [
        'robot_ip', 'robot_socket', 'own_socket', '_timeout', '_sock',
        '_counter', 'robot_address', '__dict_index'
    ]

    def __init__(self,
                 robot_ip="172.31.1.147",
                 robot_socket=30300,
                 own_socket=30333,
                 timeout=3.0):
        # IP of robot
        self.robot_ip = robot_ip
        # socket where robot expecting messages
        self.robot_socket = robot_socket
        # socket on pc runing this. Must match socket specified in Sunrise
        self.own_socket = own_socket

        self.__dict_index = ("time stamp", "num packets received",
                             "num valid packets", "error ID", "AutExt_Active",
                             "AutExt_AppReadyToStart", "DefaultApp_Error",
                             "Station_Error", "state", "App_Start",
                             "App_Enable")

        # initialize socket and bind to destination port specified in Sunrise
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self._sock.bind(
            ('0.0.0.0',
             self.own_socket))  # https://stackoverflow.com/a/2694244
        self._sock.settimeout(timeout)
        self.robot_address = (self.robot_ip, self.robot_socket)

        # set counter to correct value
        self._counter = 0
        self.update_counter()

    @property
    def timeout(self):
        return self._sock.timeout

    @timeout.setter
    def timeout(self, value):
        self._sock.settimeout(value)
        self._timeout = value

    def send_message(self, content):
        self._counter += 1
        message = "{};{};{}".format(int(time.time() * 1000), self._counter,
                                    content)
        # print(message) # debug
        message = bytes(str(message).encode("utf-8"))
        self._sock.sendto(message, self.robot_address)

    def get_state(self):
        # request robot state
        self.send_message("Get_State;true")

        # receive answer state message
        addr = (0, 0)
        while addr[0] != self.robot_ip:
            data, addr = self._sock.recvfrom(1024)

        message = data.decode("utf-8")
        message = message.split(";")
        message = dict(zip(self.__dict_index, message))
        return message

    def update_counter(self):
        self._counter = int(self.get_state()["num valid packets"])
