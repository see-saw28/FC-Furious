#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project: SAPHIRE - 201
Program name: psl_observe.py
Author: Bruno DENIS
Date created: 20201119
Purpose: basic usage of psl_package to log on screen localization data sent
         from ssl-vision

Revision History :

Date      Author      Ref  Revision (Date in YYYYMMDD format)

"""

from psl_package import paris_saclay_league as psl
import time


vision = psl.SSLVisionClient(ip='224.5.23.2', port=10020)
vision.connect3()

startTime = time.time()

try:
    while (time.time() - startTime) < 0.5 : # main loop

        time.sleep(0.01) # wait 
        print(1)
        # Asynchrone read of vision data
        data, addr = vision.sock.recvfrom(16)
        # balls: list of tuple
        #   List of ball locations. Each location is 9-uple of
        #    frame_number : int, number of camera capture
        #    t_capture : float, time of camera capture (s)
        #    t_sent : float, time of message sending
        #    camera_id : int, camera identifier
        #    confidence : float, confidence of image recognition
        #    area : int, ???
        #    x : float, x-coordonate of ball center (mm)
        #    y : float, y-coordonate of ball center (mm)
        #    z : float, z-coordonate of ball center (mm)
        #
        # blueBots, yellowBots: list of tuple
        #    List of blue robot locations. Each location is 9-uple of
        #    frame_number : int, number of camera capture
        #    t_capture : float, time of camera capture (s)
        #    t_sent : float, time of message sending
        #    camera_id : int, camera identifier
        #    confidence : float, confidence of image recognition
        #    robot_id : int, robot identifier
        #    x : float, x-coordonate of robot center (mm)
        #    y : float, y-coordonate of robot center (mm)
        #    orientation : float, orientation robot (rad)
        # for ball in balls:
        #     print(vision.ballLineStr.format(*ball))
        # for bot in blueBots:
        #     print(vision.bBotLineStr.format(*bot))        
        # for bot in yellowBots:
        #     print(vision.yBotLineStr.format(*bot))            
        # print('---')
        
 
except KeyboardInterrupt:
    pass

vision.close()

