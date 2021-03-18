#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project: SAPHIRE - 201 - Robocup
Program name: paris_saclay_league.py
Author: Bruno DENIS
Date created: 20191114
Purpose: Classes to communicate with ssl-vision and grSim

Revision History :

Date      Author      Ref  Revision (Date in YYYYMMDD format)
20200427  B. Denis      1  add isDataReady() to SSLVisionClient class
20201119  B. Denis      2  comment improvements
20201119  B. Denis      3  add getLocations() method on SSLVisionClient class
20201120  B. Denis      4  add packetCommandBot() function
"""
import socket
import struct
import select
import time
from . import messages_robocup_ssl_wrapper_pb2 as vision_pb2
from . import grSim_Packet_pb2 as grSim_pb2


class SSLVisionClient:
    """
    Class of communication object to read and decode locatization data
    sent from ssl-vision software.
    
    """  
    def __init__(self, ip = '224.5.23.2', port=10006):
        """
        Initialise SSLVisionClient object.

        Parameters
        ----------
        ip : str, optional
            Multicast IP in format '255.255.255.255'.
            The default is '224.5.23.2'.
        port : int, optional
            Port up to 1024. The default is 10006.

        Returns
        -------
        None.

        """
        self.ip = ip
        self.port = port
        self.sock = None # self.sock is set by self.connect()
        
        self.ballLineStr = 'BALL fn={0} tc={1:.3f} cam={3}'
        self.ballLineStr += ' x={6:+7.1f} y={7:+7.1f} z={8:+6.1f}'
        self.ballLineStr += ' conf={4:.3f}'

        self.bBotLineStr = 'BL{5:02d} fn={0} tc={1:.3f} cam={3}'
        self.bBotLineStr += ' x={6:+7.1f} y={7:+7.1f} o={8:+6.3f}'
        self.bBotLineStr += ' conf={4:.3f}'
        
        self.yBotLineStr = 'YE{5:02d} fn={0} tc={1:.3f} cam={3}'
        self.yBotLineStr += ' x={6:+7.1f} y={7:+7.1f} o={8:+6.3f}'
        self.yBotLineStr += ' conf={4:.3f}'


    def connect(self):
        """
        Binds a multicast group with ip and port.
        (Version proposed by Bruno Denis) 

        Returns
        -------
        None.

        """
        self.sock = socket.socket(socket.AF_INET, # family address : internet
                                  socket.SOCK_DGRAM, # type of socket : UDP
                                  socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.ip, self.port))
        mreq = struct.pack("4sl", 
                           socket.inet_aton(self.ip), 
                           socket.INADDR_ANY)
        self.sock.setsockopt(socket.IPPROTO_IP, 
                              socket.IP_ADD_MEMBERSHIP, 
                              mreq)


    def connect2(self):
        """
        Binds a multicast group with ip and port.
        Version proposed by Júnior Lima (https://github.com/Juniorlimaivd)
        
        Returns
        -------
        None.

        """
        if not isinstance(self.ip, str):
            raise ValueError('IP type should be string type')
        if not isinstance(self.port, int):
            raise ValueError('Port type should be int type')
            
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 
                                  socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 128)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        self.sock.bind((self.ip, self.port))

        host = socket.gethostbyname(socket.gethostname())
        self.sock.setsockopt(socket.SOL_IP, socket.IP_MULTICAST_IF, 
                             socket.inet_aton(host))
        self.sock.setsockopt(socket.SOL_IP, socket.IP_ADD_MEMBERSHIP, 
                socket.inet_aton(self.ip) + socket.inet_aton(host))


    def receive(self):
        """
        Read received packets and decode the Protocol Buffer message.

        Returns
        -------
        decoded_data : Protocol Buffer message
            DESCRIPTION.

        """
        data, addr = self.sock.recvfrom(1024)
        decoded_data = vision_pb2.SSL_WrapperPacket().FromString(data)
        return decoded_data


    def close(self):
        """
        Close UDP multicast socket.
        
        """
        self.sock.close()
        
        
    def isDataReady(self):
        """
        Test if data are ready to be read and decoded

        Returns
        -------
        bool
            true if data has arrived from vision software, otherwise false

        """
        rlist, wlist, xlist = select.select([self.sock], [], [], 0.01)
        return len(rlist) != 0
        
        
    def getLocations(self):
        """
        Read en decode all received message and build 3 lists.
        A list of dated location of balls
        A list of dated locations of bleu team robots
        A list of dated locations of bleu team robots

        Returns
        -------
        ball : list of tuple
            List of ball locations. Each location is 9-uple of
            frame_number : int, number of camera capture
            t_capture : float, time of camera capture (s)
            t_sent : float, time of message sending
            camera_id : int, camera identifier
            confidence : float, confidence of image recognition
            area : int, ???
            x : float, x-coordonate of ball center (mm)
            y : float, y-coordonate of ball center (mm)
            z : float, z-coordonate of ball center (mm)
        bBot : list of tuple
            List of blue robot locations. Each location is 9-uple of
            frame_number : int, number of camera capture
            t_capture : float, time of camera capture (s)
            t_sent : float, time of message sending
            camera_id : int, camera identifier
            confidence : float, confidence of image recognition
            robot_id : int, robot identifier
            x : float, x-coordonate of robot center (mm)
            y : float, y-coordonate of robot center (mm)
            orientation : float, orientation robot (rad)
        yBot : list of tuple
            List of yellow robot locations. Each location is 9-uple of
            frame_number : int, number of camera capture
            t_capture : float, time of camera capture (s)
            t_sent : float, time of message sending
            camera_id : int, camera identifier
            confidence : float, confidence of image recognition
            robot_id : int, robot identifier
            x : float, x-coordonate of robot center (mm)
            y : float, y-coordonate of robot center (mm)
            orientation : float, orientation robot (rad)

        """
        ball, bBot, yBot = list(), list(), list()
        while self.isDataReady():
            data = self.receive()
            if data.HasField('detection'):
                fn = data.detection.frame_number
                tc = data.detection.t_capture
                ts = data.detection.t_sent
                camId = data.detection.camera_id
                for b in data.detection.balls:
                    ball.append((fn, tc, ts, camId,
                                 b.confidence, b.area, 
                                 b.x, b.y, b.z))
                for r in data.detection.robots_blue:
                    bBot.append((fn, tc, ts, camId, 
                                 r.confidence, r.robot_id, 
                                 r.x, r.y, r.orientation))
                for r in data.detection.robots_yellow:
                    yBot.append((fn, tc, ts, camId, 
                                 r.confidence, r.robot_id, 
                                 r.x, r.y, r.orientation))
        return ball, bBot, yBot

class SSLgrSimClient:

    def __init__(self, ip = '127.0.0.1', port=20011):
        """
        Initialisation of SSLgrSimClient object.

        Parameters
        ----------
        ip : str, optional
            Multicast IP in format '255.255.255.255'. 
            The default is '127.0.0.1'.
        port : int, optional
            Port up to 1024. The default is 20011.

        Returns
        -------
        None.

        """
        self.ip = ip
        self.port = port
        
        
    def connect(self):
        """
        Open UDP unicast socket.

        Returns
        -------
        None.

        """
        self.sock = socket.socket(socket.AF_INET, # family address : internet
                                  socket.SOCK_DGRAM) # type of socket : UDP


    def send(self, packet):
        """
        Encode and send command data.

        Parameters
        ----------
        packet : TYPE
            DESCRIPTION.

        Returns
        -------
        None.

        """
        serialized_data = packet.SerializeToString()
        #serialized_data = packet.SerializePartialToString()
        self.sock.sendto(serialized_data, (self.ip, self.port))
        
        
    def close(self):
        """
        Close UDP unicast socket.

        Returns
        -------
        None.

        """
        self.sock.close()
        
        
def packetCommandBot(isYellow, id, 
                     veltangent, velnormal, velangular,
                     spinner=False, kickspeedx=0, kickspeedz=0):
    """
    Create a packet (according the Protocol Buffers description)
    containing a robot command.

    Parameters
    ----------
    isYellow : bool
        True if the robot is yellow otherwise False
    id : interger
        Robot identifier
    veltangent : float
        Tangent velocity (m/s)
    velnormal : float
        Normal velocity (m/s)
    velangular : float
        Angular velocity (rad/s).
    spinner : bool, optionals
        Set th spinner on (dribbler) if True
    kickspeedx : float, optional
        DESCRIPTION. The default is 0.
    kickspeedz : float, optional
        DESCRIPTION. The default is 0.

    Returns
    -------
    packet : grSim_Packet_pb2.grSim_Packet

    """
    packet = grSim_pb2.grSim_Packet()
    packet.commands.timestamp = time.monotonic()
    packet.commands.isteamyellow = isYellow
    robot_commands = packet.commands.robot_commands.add()
    robot_commands.id = id
    robot_commands.kickspeedx = kickspeedx
    robot_commands.kickspeedz = kickspeedz
    robot_commands.veltangent = veltangent
    robot_commands.velnormal = velnormal
    robot_commands.velangular = velangular
    robot_commands.spinner = spinner
    robot_commands.wheelsspeed = False
    return packet
