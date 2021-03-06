#!/usr/bin/python3

import sys
import enum
import math
import time
import json

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from argos_bridge.msg import ProximityList
from argos_bridge.msg import BaseGroundList
from argos_bridge.msg import MotoGroundList
from argos_bridge.msg import PuckList
from argos_bridge.msg import Position
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String

class ZoneColor(enum.IntEnum):
    BLACK = 0
    RED = 1
    BLUE = 2
    GREEN = 3
    YELLOW = 4

class PuckColor(enum.IntEnum):
    UNRECOGNIZED = -2
    NONPUCK = -1
    RED = 0
    BLUE = 1
    GREEN = 2
    YELLOW = 3
    
class Zone():
    def __init__(self):
        self.zoneColor = ZoneColor.BLACK
        self.zoneNumber = 0
        self.isInZone = False
        
    def SetZoneColor(self, color: ZoneColor):
        self.zoneColor = color

    def SetZoneNumber(self, number):
        self.zoneNumber = number

    def SetIsInZoneFlage(self, inZone: Bool):
        self.isInZone = inZone

    def GetZoneColor(self):
        return self.zoneColor

    def GetZoneNumber(self):
        return self.zoneNumber

    def IsInZone(self):
        return self.isInZone

    
        
class TransportRobot:
    def __init__(self, botNumber):
        self.botNumber = botNumber
        self.botName = 'bot' + str(botNumber)
        self.robotStop = False
        
        self.isInZone = False
        self.zone = Zone()
        self.puckColor = PuckColor.NONPUCK 
        
        self.pubMove = rospy.Publisher("/" + self.botName + "/cmd_vel", Twist, queue_size=10)
        self.pubGripper = rospy.Publisher("/" + self.botName + "/gripper", Bool, queue_size=10)
        self.pubTurret = rospy.Publisher("/" + self.botName + "/turretController", Float32, queue_size=10)
        self.pubRobotAnswer = rospy.Publisher("/" + self.botName + "/robotAnswer", String, queue_size=10)
        self.move = Twist()
        self.gripper = Bool(False)
        self.turret = Float32(0)

        self.puckList = PuckList()
        self.newPuckListMsg = False
        self.position = Position()
        self.newPositionMsg = False
        self.turretEncoder = Float32()
        self.newTurretEncoderMsg = False

        self.command = ''
        self.paramList = []
        self.command_first_loop = True
		
        rospy.Subscriber("/" + self.botName + "/puck_list", PuckList, self.callbackPuckList)
        rospy.Subscriber("/" + self.botName + "/position", Position, self.callbackPosition)
        rospy.Subscriber("/" + self.botName + "/robotStop", Bool, self.callbackRobotStop)
        rospy.Subscriber("/" + self.botName + "/turretEncoder", Float32, self.callbackTurretEncoder)

        while self.newPuckListMsg == False:
            pass

        rospy.Subscriber("/" + self.botName + "/robotController", String, self.callbackRobotController)

    def callbackPuckList(self, msg):
        self.puckList = msg
        self.newPuckListMsg = True

    def callbackPosition(self, msg):
        self.position = msg
        self.newPositionMsg = True
		
        if self.command == 'moveToPosition':
            if self.MoveToPositionNotBlock(self.paramList[0], self.paramList[1], 0.1, 0.025, 1, 1):
                self.command = ''
                answer = 'moveToPosition'
                x = self.position.position.x
                y = self.position.position.y
                orientation = self.position.orientation.z
                self.pubRobotAnswer.publish(self.code_command(self.botNumber, answer, [x, y, orientation]))

        elif self.command == 'takeNearestPuck':
            if self.command_first_loop:
                [self.puckPositionX, self.puckPositionY] = self.CalculateNearestPuckPosition()

                if self.puckPositionX == 1 and self.puckPositionY == 1:
                    self.command = ''
                    answer = 'takeNearestPuck'
                    self.pubRobotAnswer.publish(self.code_command(self.botNumber, answer, [PuckColor.NONPUCK]))
                    return

                self.command_first_loop = False

            if self.MoveToPositionNotBlock(self.puckPositionX, self.puckPositionY, 0.05, 0.16, 1, 1):
                self.command = ''
                answer = 'takeNearestPuck'
                self.gripper = True
                self.command_first_loop = True
                self.pubGripper.publish(self.gripper)
                self.pubRobotAnswer.publish(self.code_command(self.botNumber, answer, [self.GetPuckColor()]))

        elif self.command == 'escapeFromZone':
            if self.zone.IsInZone():
                if self.command_first_loop:

                    self.escapePositionX, self.escapePositionY, self.escapeOrientation = self.GetZoneTagPositionAndOrientation()

                    if math.isinf(self.escapePositionX):
                        print("unknown zone color or number")
                        self.command = ''
                        answer = 'escapeFromZone'
                        self.pubRobotAnswer.publish(self.code_command(self.botNumber, answer, [False]))

                    self.command_first_loop = False

                if self.MoveToPositionEscapeZoneNotBlock(self.escapePositionX, self.escapePositionY, self.escapeOrientation, 0.6, 1, 1):
                    self.command = ''
                    answer = 'escapeFromZone'
                    x = self.position.position.x
                    y = self.position.position.y
                    orientation = self.position.orientation.z
                    self.command_first_loop = True
                    self.zone.SetIsInZoneFlage(False)
                    self.pubRobotAnswer.publish(self.code_command(self.botNumber, answer, [x, y, orientation]))

            else:
                self.command = ''
                answer = 'escapeFromZone'
                self.pubRobotAnswer.publish(self.code_command(self.botNumber, answer, [False]))

        elif self.command == 'putDownPuckOnPosition':
            if self.command_first_loop:
                if not self.CheckPositionIsEmpty(self.paramList[0], self.paramList[1]):
                    self.command = ''
                    answer = 'putDownPuckOnPosition'
                    self.pubRobotAnswer.publish(self.code_command(self.botNumber, answer, [False]))

                self.command_first_loop = False

            if self.MoveToPositionNotBlock(self.paramList[0], self.paramList[1], 0.05, 0.14, 1, 1):
                self.gripper = False
                self.pubGripper.publish(self.gripper)
                self.command = ''
                answer = 'putDownPuckOnPosition'
                self.command_first_loop = True
                self.pubRobotAnswer.publish(self.code_command(self.botNumber, answer, [True]))


    def callbackRobotStop(self, msg):
        self.robotStop = msg.data

    def callbackTurretEncoder(self, msg):
        self.turretEncoder = msg
        self.newTurretEncoderMsg = True

    def callbackRobotController(self, msg):
        robotNumber, command, paramList = self.decode_command(msg.data)
        print(msg)
        answer = 'not found'
        answerParam = []
        if command == 'enterToZone' and len(paramList) == 1:
            answer = 'enterToZone'
            answerParam.append(self.EnterToZone(paramList[0]))

        elif command == 'takeNearestPuck':
            answer = 'takeNearestPuck'
            self.command = 'takeNearestPuck'
            #answerParam.append(self.TakeNearestPuck())
            return

        elif command == 'escapeFromZone':
            answer = 'escapeFromZone'
            self.command = 'escapeFromZone'
            #answerParam = self.EscapeFromZone()
            return

        elif command == 'putDownPuckOnPosition' and len(paramList) == 2:
            answer = 'putDownPuckOnPosition'
            self.command = 'putDownPuckOnPosition'
            self.paramList = paramList
            return
            #answerParam.append(self.PutDownPuckOnPosition(paramList[0], paramList[1]))

        elif command == 'getPuckColor':
            answer = 'getPuckColor'
            answerParam.append(self.GetPuckColor())

        elif command == 'getOrientation':
            answer = 'getOrientation'
            answerParam.append(self.position.orientation.z)

        elif command == 'moveToPosition' and len(paramList) == 2:
            #answer = 'moveToPosition'
            self.command = 'moveToPosition'
            self.paramList = paramList
            return
            #answerParam = self.MoveToPosition(paramList[0], paramList[1], 0.1, 0.025, 1, 1)

        self.pubRobotAnswer.publish(self.code_command(self.botNumber, answer, answerParam))

    def decode_command(self, sentence):
        command = json.loads(sentence)
        return int(command['robot']), command['name'], command['values']

    def code_command(self, robot_number, command_name, paramList):
        value = []
        for item in paramList:
            value.append(item)
        code = json.dumps({"robot": robot_number, "name": command_name, "values": value})
        return code

    def GetPuckColor(self):
        return self.puckColor

    def MoveToPosition(self, x, y, rangeCanDriveBackward, accuracyRange,  SpeedScale, SpeedScaleRot):
        if accuracyRange < 0.001:
            accuracyRange = 0.001

        run = True
        while run and not self.robotStop:
            if self.newPositionMsg:
                self.newPositionMsg = False

                [angle, range] = self.CalculateAngleAndRangeToPosition(x, y)

                if range < accuracyRange:
                    self.move.linear.x = 0
                    self.move.angular.z = 0
                    run = False

                elif range > rangeCanDriveBackward:
                    if angle > 0.1:
                        self.move.linear.x = 0
                        self.move.angular.z = -2.5 * SpeedScaleRot

                    elif angle < -0.1:
                        self.move.linear.x = 0
                        self.move.angular.z = 2.5 * SpeedScaleRot

                    elif angle > 0.03:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.5 * SpeedScaleRot

                    elif angle < -0.03:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.5 * SpeedScaleRot

                    elif angle > 0.01:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.1 * SpeedScaleRot

                    elif angle < -0.01:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.1 * SpeedScaleRot

                    elif range >= 0.05:
                        self.move.linear.x = 2 * SpeedScale
                        self.move.angular.z = 0

                elif range <= rangeCanDriveBackward:

                    if angle < 0.01 and angle > -0.01:
                        self.move.angular.z = 0

                        if range >= 0.1:
                            self.move.linear.x = 2 * SpeedScale

                        elif  range >= 0.05:
                            self.move.linear.x = 1 * SpeedScale

                        elif range >= 0.01:
                            self.move.linear.x = 0.1 * SpeedScale

                        elif range >= 0.001:
                            self.move.linear.x = 0.05 * SpeedScale

                    elif (angle > math.pi-0.01 and angle < math.pi+0.01
                          or angle < -math.pi+0.01 and angle > -math.pi-0.01):
                        self.move.angular.z = 0

                        if range >= 0.1:
                            self.move.linear.x = -2 * SpeedScale

                        elif range >= 0.05:
                            self.move.linear.x = -1 * SpeedScale

                        elif range >= 0.01:
                            self.move.linear.x = -0.1 * SpeedScale

                        elif range >= 0.001:
                            self.move.linear.x = -0.05 * SpeedScale


                    elif angle >= 0.01 and angle < 0.03:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.1 * SpeedScaleRot

                    elif angle < -0.01 and angle > -0.03:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.1 * SpeedScaleRot

                    elif angle >= 0.03 and angle < 0.1:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.5 * SpeedScaleRot

                    elif angle <= -0.03 and angle > -0.1:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.5 * SpeedScaleRot

                    elif angle >= 0.1 and angle < math.pi/2:
                        self.move.linear.x = 0
                        self.move.angular.z = -2.5 * SpeedScaleRot

                    elif angle <= -0.1 and angle > -math.pi/2:
                        self.move.linear.x = 0
                        self.move.angular.z = 2.5 * SpeedScaleRot

                    elif angle <= math.pi-0.01 and angle >= math.pi-0.03 or angle < -math.pi-0.01 and angle >= -math.pi-0.03:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.1 * SpeedScaleRot

                    elif angle >= -math.pi+0.01 and angle <= -math.pi+0.03 or angle > math.pi+0.03 and angle <= math.pi+0.01:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.1 * SpeedScaleRot

                    elif angle <= math.pi-0.03 and angle > math.pi-0.1 or angle < -math.pi-0.03 and angle >= -math.pi-0.1:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.5 * SpeedScaleRot

                    elif angle >= -math.pi+0.03 and angle < -math.pi+0.1 or angle > math.pi+0.03 and angle <= math.pi+0.01:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.5 * SpeedScaleRot

                    elif angle <= math.pi-0.1 and angle >= math.pi/2 or angle < -math.pi-0.1 and angle >= -3*math.pi/2:
                        self.move.linear.x = 0
                        self.move.angular.z = 2.5 * SpeedScaleRot

                    elif angle >= -math.pi+0.1 and angle <= -math.pi/2 or angle > math.pi+0.1 and angle <= 3*math.pi/2:
                        self.move.linear.x = 0
                        self.move.angular.z = -2.5 * SpeedScaleRot

                self.pubMove.publish(self.move)

        x = self.position.position.x
        y = self.position.position.y
        orientation = self.position.orientation.z

        return x, y, orientation

    def MoveToPositionNotBlock(self, x, y, rangeCanDriveBackward, accuracyRange,  SpeedScale, SpeedScaleRot):
        if accuracyRange < 0.001:
            accuracyRange = 0.001

        [angle, range] = self.CalculateAngleAndRangeToPosition(x, y)
        finish = False
        if range < accuracyRange:
            self.move.linear.x = 0
            self.move.angular.z = 0
            finish = True

        elif range > rangeCanDriveBackward:
            if angle > 0.1:
                self.move.linear.x = 0
                self.move.angular.z = -2.5 * SpeedScaleRot

            elif angle < -0.1:
                self.move.linear.x = 0
                self.move.angular.z = 2.5 * SpeedScaleRot

            elif angle > 0.03:
                self.move.linear.x = 0
                self.move.angular.z = -0.5 * SpeedScaleRot

            elif angle < -0.03:
                self.move.linear.x = 0
                self.move.angular.z = 0.5 * SpeedScaleRot

            elif angle > 0.01:
                self.move.linear.x = 0
                self.move.angular.z = -0.1 * SpeedScaleRot

            elif angle < -0.01:
                self.move.linear.x = 0
                self.move.angular.z = 0.1 * SpeedScaleRot

            elif range >= 0.05:
                self.move.linear.x = 2 * SpeedScale
                self.move.angular.z = 0

        elif range <= rangeCanDriveBackward:

            if angle < 0.01 and angle > -0.01:
                self.move.angular.z = 0

                if range >= 0.1:
                    self.move.linear.x = 2 * SpeedScale

                elif range >= 0.05:
                    self.move.linear.x = 1 * SpeedScale

                elif range >= 0.01:
                    self.move.linear.x = 0.1 * SpeedScale

                elif range >= 0.001:
                    self.move.linear.x = 0.05 * SpeedScale

            elif (angle > math.pi - 0.01 and angle < math.pi + 0.01
                  or angle < -math.pi + 0.01 and angle > -math.pi - 0.01):
                self.move.angular.z = 0

                if range >= 0.1:
                    self.move.linear.x = -2 * SpeedScale

                elif range >= 0.05:
                    self.move.linear.x = -1 * SpeedScale

                elif range >= 0.01:
                    self.move.linear.x = -0.1 * SpeedScale

                elif range >= 0.001:
                    self.move.linear.x = -0.05 * SpeedScale


            elif angle >= 0.01 and angle < 0.03:
                self.move.linear.x = 0
                self.move.angular.z = -0.1 * SpeedScaleRot

            elif angle < -0.01 and angle > -0.03:
                self.move.linear.x = 0
                self.move.angular.z = 0.1 * SpeedScaleRot

            elif angle >= 0.03 and angle < 0.1:
                self.move.linear.x = 0
                self.move.angular.z = -0.5 * SpeedScaleRot

            elif angle <= -0.03 and angle > -0.1:
                self.move.linear.x = 0
                self.move.angular.z = 0.5 * SpeedScaleRot

            elif angle >= 0.1 and angle < math.pi / 2:
                self.move.linear.x = 0
                self.move.angular.z = -2.5 * SpeedScaleRot

            elif angle <= -0.1 and angle > -math.pi / 2:
                self.move.linear.x = 0
                self.move.angular.z = 2.5 * SpeedScaleRot

            elif angle <= math.pi - 0.01 and angle >= math.pi - 0.03 or angle < -math.pi - 0.01 and angle >= -math.pi - 0.03:
                self.move.linear.x = 0
                self.move.angular.z = 0.1 * SpeedScaleRot

            elif angle >= -math.pi + 0.01 and angle <= -math.pi + 0.03 or angle > math.pi + 0.03 and angle <= math.pi + 0.01:
                self.move.linear.x = 0
                self.move.angular.z = -0.1 * SpeedScaleRot

            elif angle <= math.pi - 0.03 and angle > math.pi - 0.1 or angle < -math.pi - 0.03 and angle >= -math.pi - 0.1:
                self.move.linear.x = 0
                self.move.angular.z = 0.5 * SpeedScaleRot

            elif angle >= -math.pi + 0.03 and angle < -math.pi + 0.1 or angle > math.pi + 0.03 and angle <= math.pi + 0.01:
                self.move.linear.x = 0
                self.move.angular.z = -0.5 * SpeedScaleRot

            elif angle <= math.pi - 0.1 and angle >= math.pi / 2 or angle < -math.pi - 0.1 and angle >= -3 * math.pi / 2:
                self.move.linear.x = 0
                self.move.angular.z = 2.5 * SpeedScaleRot

            elif angle >= -math.pi + 0.1 and angle <= -math.pi / 2 or angle > math.pi + 0.1 and angle <= 3 * math.pi / 2:
                self.move.linear.x = 0
                self.move.angular.z = -2.5 * SpeedScaleRot

        self.pubMove.publish(self.move)

        return finish

    def MoveToPositionEscapeZone(self, x, y, orientation, rangeCanDriveBackward, SpeedScale, SpeedScaleRot):

        run = True
        while run and not self.robotStop:
            if self.newPositionMsg:
                self.newPositionMsg = False

                [angle, range] = self.CalculateAngleAndRangeToPosition(x, y)

                if range < 0.01:
                    self.move.linear.x = 0
                    self.move.angular.z = 0
                    run = False

                elif range > rangeCanDriveBackward:
                    if angle > 0.1:
                        self.move.linear.x = 0
                        self.move.angular.z = -2.5 * SpeedScaleRot

                    elif angle < -0.1:
                        self.move.linear.x = 0
                        self.move.angular.z = 2.5 * SpeedScaleRot

                    elif angle > 0.03:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.5 * SpeedScaleRot

                    elif angle < -0.03:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.5 * SpeedScaleRot

                    elif angle > 0.01:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.1 * SpeedScaleRot

                    elif angle < -0.01:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.1 * SpeedScaleRot

                    elif range >= 0.05:
                        self.move.linear.x = 2 * SpeedScale
                        self.move.angular.z = 0

                elif range <= rangeCanDriveBackward:

                    if angle < 0.01 and angle > -0.01:
                        self.move.angular.z = 0

                        if range >= 0.1:
                            self.move.linear.x = 2 * SpeedScale

                        elif range >= 0.05:
                            self.move.linear.x = 1 * SpeedScale

                        elif range >= 0.01:
                            self.move.linear.x = 0.1 * SpeedScale

                        elif range >= 0.001:
                            self.move.linear.x = 0.05 * SpeedScale

                    elif (angle > math.pi - 0.01 and angle < math.pi + 0.01
                          or angle < -math.pi + 0.01 and angle > -math.pi - 0.01):
                        self.move.angular.z = 0

                        if range >= 0.1:
                            self.move.linear.x = -2 * SpeedScale

                        elif range >= 0.05:
                            self.move.linear.x = -1 * SpeedScale

                        elif range >= 0.01:
                            self.move.linear.x = -0.1 * SpeedScale

                        elif range >= 0.001:
                            self.move.linear.x = -0.05 * SpeedScale


                    elif angle >= 0.01 and angle < 0.03:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.1 * SpeedScaleRot

                    elif angle < -0.01 and angle > -0.03:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.1 * SpeedScaleRot

                    elif angle >= 0.03 and angle < 0.1:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.5 * SpeedScaleRot

                    elif angle <= -0.03 and angle > -0.1:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.5 * SpeedScaleRot

                    elif angle >= 0.1 and angle < math.pi / 2:
                        self.move.linear.x = 0
                        self.move.angular.z = -2.5 * SpeedScaleRot

                    elif angle <= -0.1 and angle > -math.pi / 2:
                        self.move.linear.x = 0
                        self.move.angular.z = 2.5 * SpeedScaleRot

                    elif angle <= math.pi - 0.01 and angle > math.pi - 0.03 or angle < -math.pi - 0.01 and angle >= -math.pi - 0.03:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.1 * SpeedScaleRot

                    elif angle >= -math.pi + 0.01 and angle < -math.pi + 0.03 or angle > math.pi + 0.03 and angle <= math.pi + 0.01:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.1 * SpeedScaleRot

                    elif angle <= math.pi - 0.03 and angle > math.pi - 0.1 or angle < -math.pi - 0.03 and angle >= -math.pi - 0.1:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.5 * SpeedScaleRot

                    elif angle >= -math.pi + 0.03 and angle < -math.pi + 0.1 or angle > math.pi + 0.03 and angle <= math.pi + 0.01:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.5 * SpeedScaleRot

                    elif angle <= math.pi - 0.1 and angle >= math.pi / 2 or angle < -math.pi - 0.1 and angle >= -3 * math.pi / 2:
                        self.move.linear.x = 0
                        self.move.angular.z = 2.5 * SpeedScaleRot

                    elif angle >= -math.pi + 0.1 and angle <= -math.pi / 2 or angle > math.pi + 0.1 and angle <= 3 * math.pi / 2:
                        self.move.linear.x = 0
                        self.move.angular.z = -2.5 * SpeedScaleRot

                self.pubMove.publish(self.move)

        run = True
        while run and not self.robotStop:
            if self.newPositionMsg:
                self.newPositionMsg = False

                angle = self.position.orientation.z - orientation

                if angle < -math.pi:
                    angle = angle + 2 * math.pi

                elif angle > math.pi:
                    angle = angle - 2 * math.pi

                #print('angle: {}, rob_ort: {}, finish_ort: {}'.format(angle, self.position.orientation.z, orientation))

                if angle > 0.1:
                    self.move.linear.x = 0
                    self.move.angular.z = -2.5 * SpeedScaleRot

                elif angle < -0.1:
                    self.move.linear.x = 0
                    self.move.angular.z = 2.5 * SpeedScaleRot

                elif angle > 0.03:
                    self.move.linear.x = 0
                    self.move.angular.z = -0.5 * SpeedScaleRot

                elif angle < -0.03:
                    self.move.linear.x = 0
                    self.move.angular.z = 0.5 * SpeedScaleRot

                elif angle > 0.01:
                    self.move.linear.x = 0
                    self.move.angular.z = -0.1 * SpeedScaleRot

                elif angle < -0.01:
                    self.move.linear.x = 0
                    self.move.angular.z = 0.1 * SpeedScaleRot

                elif angle <= 0.01 and angle >= -0.01:
                    self.move.linear.x = 0
                    self.move.angular.z = 0
                    run = False

                self.pubMove.publish(self.move)

        x = self.position.position.x
        y = self.position.position.y
        orientation = self.position.orientation.z

        return x, y, orientation

    def MoveToPositionEscapeZoneNotBlock(self, x, y, orientation, rangeCanDriveBackward, SpeedScale, SpeedScaleRot):
        finish = False
        rotate = False

        [angle, range] = self.CalculateAngleAndRangeToPosition(x, y)

        if range < 0.01:
            self.move.linear.x = 0
            self.move.angular.z = 0
            rotate = True

        elif range > rangeCanDriveBackward:
            if angle > 0.1:
                self.move.linear.x = 0
                self.move.angular.z = -2.5 * SpeedScaleRot

            elif angle < -0.1:
                self.move.linear.x = 0
                self.move.angular.z = 2.5 * SpeedScaleRot

            elif angle > 0.03:
                self.move.linear.x = 0
                self.move.angular.z = -0.5 * SpeedScaleRot

            elif angle < -0.03:
                self.move.linear.x = 0
                self.move.angular.z = 0.5 * SpeedScaleRot

            elif angle > 0.01:
                self.move.linear.x = 0
                self.move.angular.z = -0.1 * SpeedScaleRot

            elif angle < -0.01:
                self.move.linear.x = 0
                self.move.angular.z = 0.1 * SpeedScaleRot

            elif range >= 0.05:
                self.move.linear.x = 2 * SpeedScale
                self.move.angular.z = 0

        elif range <= rangeCanDriveBackward:

            if angle < 0.01 and angle > -0.01:
                self.move.angular.z = 0

                if range >= 0.1:
                    self.move.linear.x = 2 * SpeedScale

                elif range >= 0.05:
                    self.move.linear.x = 1 * SpeedScale

                elif range >= 0.01:
                    self.move.linear.x = 0.1 * SpeedScale

                elif range >= 0.001:
                    self.move.linear.x = 0.05 * SpeedScale

            elif (angle > math.pi - 0.01 and angle < math.pi + 0.01
                  or angle < -math.pi + 0.01 and angle > -math.pi - 0.01):
                self.move.angular.z = 0

                if range >= 0.1:
                    self.move.linear.x = -2 * SpeedScale

                elif range >= 0.05:
                    self.move.linear.x = -1 * SpeedScale

                elif range >= 0.01:
                    self.move.linear.x = -0.1 * SpeedScale

                elif range >= 0.001:
                    self.move.linear.x = -0.05 * SpeedScale


            elif angle >= 0.01 and angle < 0.03:
                self.move.linear.x = 0
                self.move.angular.z = -0.1 * SpeedScaleRot

            elif angle < -0.01 and angle > -0.03:
                self.move.linear.x = 0
                self.move.angular.z = 0.1 * SpeedScaleRot

            elif angle >= 0.03 and angle < 0.1:
                self.move.linear.x = 0
                self.move.angular.z = -0.5 * SpeedScaleRot

            elif angle <= -0.03 and angle > -0.1:
                self.move.linear.x = 0
                self.move.angular.z = 0.5 * SpeedScaleRot

            elif angle >= 0.1 and angle < math.pi / 2:
                self.move.linear.x = 0
                self.move.angular.z = -2.5 * SpeedScaleRot

            elif angle <= -0.1 and angle > -math.pi / 2:
                self.move.linear.x = 0
                self.move.angular.z = 2.5 * SpeedScaleRot

            elif angle <= math.pi - 0.01 and angle > math.pi - 0.03 or angle < -math.pi - 0.01 and angle >= -math.pi - 0.03:
                self.move.linear.x = 0
                self.move.angular.z = 0.1 * SpeedScaleRot

            elif angle >= -math.pi + 0.01 and angle < -math.pi + 0.03 or angle > math.pi + 0.03 and angle <= math.pi + 0.01:
                self.move.linear.x = 0
                self.move.angular.z = -0.1 * SpeedScaleRot

            elif angle <= math.pi - 0.03 and angle > math.pi - 0.1 or angle < -math.pi - 0.03 and angle >= -math.pi - 0.1:
                self.move.linear.x = 0
                self.move.angular.z = 0.5 * SpeedScaleRot

            elif angle >= -math.pi + 0.03 and angle < -math.pi + 0.1 or angle > math.pi + 0.03 and angle <= math.pi + 0.01:
                self.move.linear.x = 0
                self.move.angular.z = -0.5 * SpeedScaleRot

            elif angle <= math.pi - 0.1 and angle >= math.pi / 2 or angle < -math.pi - 0.1 and angle >= -3 * math.pi / 2:
                self.move.linear.x = 0
                self.move.angular.z = 2.5 * SpeedScaleRot

            elif angle >= -math.pi + 0.1 and angle <= -math.pi / 2 or angle > math.pi + 0.1 and angle <= 3 * math.pi / 2:
                self.move.linear.x = 0
                self.move.angular.z = -2.5 * SpeedScaleRot

        self.pubMove.publish(self.move)

        if rotate:
            angle = self.position.orientation.z - orientation

            if angle < -math.pi:
                angle = angle + 2 * math.pi

            elif angle > math.pi:
                angle = angle - 2 * math.pi

            if angle > 0.1:
                self.move.linear.x = 0
                self.move.angular.z = -2.5 * SpeedScaleRot

            elif angle < -0.1:
                self.move.linear.x = 0
                self.move.angular.z = 2.5 * SpeedScaleRot

            elif angle > 0.03:
                self.move.linear.x = 0
                self.move.angular.z = -0.5 * SpeedScaleRot

            elif angle < -0.03:
                self.move.linear.x = 0
                self.move.angular.z = 0.5 * SpeedScaleRot

            elif angle > 0.01:
                self.move.linear.x = 0
                self.move.angular.z = -0.1 * SpeedScaleRot

            elif angle < -0.01:
                self.move.linear.x = 0
                self.move.angular.z = 0.1 * SpeedScaleRot

            elif angle <= 0.01 and angle >= -0.01:
                self.move.linear.x = 0
                self.move.angular.z = 0
                finish = True

            self.pubMove.publish(self.move)

        return finish

    def CalculateAngleAndRangeToPosition(self, x, y):
        robotPosition = self.position.position
        dX = x - robotPosition.x
        dY = y - robotPosition.y

        range = math.sqrt(dX * dX + dY * dY)

        alpha = math.atan2(dY, dX)
        robotOrientation = self.position.orientation.z
        dTheta = robotOrientation - alpha

        # if dTheta is more then 180 we won't make rotation 270 but 90 in opposite size
        if dTheta < -math.pi:
            dTheta = dTheta + 2*math.pi

        elif dTheta > math.pi:
            dTheta = dTheta - 2*math.pi

        #print('alpha: {}, orie: {}, dtheta: {}'.format(alpha, robotOrientation, dTheta))
        #print('range: {}'.format(range))


        return [dTheta, range]

    def TakeNearestPuck(self):            
        [puckPositionX, puckPositionY] = self.CalculateNearestPuckPosition()
        print("puckPositionX: {}, puckPositionY: {}".format(puckPositionX, puckPositionY))

        if puckPositionX == 1 and puckPositionY == 1:
            return PuckColor.NONPUCK

        self.MoveToPosition(puckPositionX, puckPositionY, 0.05, 0.14, 1, 1)

        self.gripper = True
        self.pubGripper.publish(self.gripper)
        return self.GetPuckColor()

    def CalculateNearestPuckPosition(self):
        x = self.position.position.x
        y = self.position.position.y
        orientation = self.position.orientation.z

        tempZoneNumber = self.zone.GetZoneNumber()

        puckListClone = self.puckList
        puckInZoneList = []


        for puck in puckListClone.pucks:
            if  (puck.color.r == 165 and puck.color.g == 42 and puck.color.b == 42) or\
                (puck.color.r == 255 and puck.color.g == 140 and puck.color.b == 0) or \
                (puck.color.r == 255 and puck.color.g == 0 and puck.color.b == 255) or \
                (puck.color.r == 0 and puck.color.g == 255 and puck.color.b == 255) or\
                (puck.color.r == 160 and puck.color.g == 32 and puck.color.b == 240):
                continue
            else:
                alpha = orientation + puck.angle

                dX = math.cos(alpha) * puck.range*0.01
                dY = math.sin(alpha) * puck.range*0.01

                #print("dX: {}, dY: {}".format(dX, dY))
                if tempZoneNumber == 1:
                    if(x + dX) < 0 and (y + dY) < 0 and (x + dX) > -0.6 and (y + dY) > -0.6:
                        puckInZoneList.append(puck)

                elif tempZoneNumber == 2:
                    if (x + dX) < 0 and (y + dY) > 0 and (x + dX) > -0.6 and (y + dY) < 0.6:
                        puckInZoneList.append(puck)

                elif tempZoneNumber == 3:
                    if (x + dX) > 0 and (y + dY) > 0 and (x + dX) < 0.6 and (y + dY) < 0.6:
                        puckInZoneList.append(puck)

                elif tempZoneNumber == 4:
                    if (x + dX) > 0 and (y + dY) < 0 and (x + dX) < 0.6 and (y + dY) > -0.6:
                        puckInZoneList.append(puck)


        if len(puckInZoneList) == 0:
            print("There are no pucks in the zone")
            return [1, 1]

        nearestPuck = -1
        minRange = 1000
        i = -1
        for puck in puckInZoneList:
            i = i + 1
            if puck.range <= minRange:
                nearestPuck = i
                minRange = puck.range

        print("r: {}, g: {}, b: {}".format(puckInZoneList[nearestPuck].color.r, puckInZoneList[nearestPuck].color.g, puckInZoneList[nearestPuck].color.b))
        self.SetPuckColor(puckInZoneList[nearestPuck].color)

        puckAngle = puckInZoneList[nearestPuck].angle
        puckRange = puckInZoneList[nearestPuck].range*0.01

        alpha = orientation + puckAngle

        dX = math.cos(alpha)*puckRange
        dY = math.sin(alpha)*puckRange

        puckPositionX = x + dX
        puckPositionY = y + dY

        #print("x: {}, y: {}, range: {}, alpha: {}".format(x,y,puckRange, alpha))
        #print("dX: {}, dY: {}".format(dX, dY))

        return [puckPositionX, puckPositionY]

    def SetPuckColor(self, color):
        if color.r == 255 and color.g == 0 and color.b == 0:
            self.puckColor = PuckColor.RED
        elif color.r == 0 and color.g == 255 and color.b == 0:
            self.puckColor = PuckColor.GREEN
        elif color.r == 0 and color.g == 0 and color.b == 255:
            self.puckColor = PuckColor.BLUE
        elif color.r == 255 and color.g == 255 and color.b == 0:
            self.puckColor = PuckColor.YELLOW
        else:
            self.puckColor = PuckColor.UNRECOGNIZED

    def EnterToZone(self, zoneColor: ZoneColor):
        self.zone.SetZoneColor(zoneColor)
        self.zone.SetIsInZoneFlage(True)
        tempZoneNumber = -1
        accuracy = 0.05
        robotPositionX = self.position.position.x
        robotPositionY = self.position.position.y

        if zoneColor == ZoneColor.BLACK:
            if (robotPositionX > -0.525 - accuracy and robotPositionX < -0.525 + accuracy) and\
               (robotPositionY > -0.3 - accuracy and robotPositionY < -0.3 + accuracy):
                tempZoneNumber = 1
                self.zone.SetZoneNumber(tempZoneNumber)

            elif (robotPositionX > -0.3 - accuracy and robotPositionX < -0.3 + accuracy) and\
               (robotPositionY > 0.525 - accuracy and robotPositionY < 0.525 + accuracy):
                tempZoneNumber = 2
                self.zone.SetZoneNumber(tempZoneNumber)

            elif (robotPositionX > 0.525 - accuracy and robotPositionX < 0.525 + accuracy) and\
               (robotPositionY > 0.3 - accuracy and robotPositionY < 0.3 + accuracy):
                tempZoneNumber = 3
                self.zone.SetZoneNumber(tempZoneNumber)

            elif (robotPositionX > 0.3 - accuracy and robotPositionX < 0.3 + accuracy) and\
               (robotPositionY > -0.525 - accuracy and robotPositionY < -0.525 + accuracy):
                tempZoneNumber = 4
                self.zone.SetZoneNumber(tempZoneNumber)

        elif zoneColor == ZoneColor.RED:
            if (robotPositionX > -2.4 - accuracy and robotPositionX < -2.4 + accuracy) and \
               (robotPositionY > 1.875 - accuracy and robotPositionY < 1.875 + accuracy):
                tempZoneNumber = 1
                self.zone.SetZoneNumber(tempZoneNumber)

            elif (robotPositionX > -1.875 - accuracy and robotPositionX < -1.875 + accuracy) and\
               (robotPositionY > 2.7 - accuracy and robotPositionY < 2.7 + accuracy):
                tempZoneNumber = 2
                self.zone.SetZoneNumber(tempZoneNumber)

        elif zoneColor == ZoneColor.GREEN:
            if (robotPositionX > -2.4 - accuracy and robotPositionX < -2.4 + accuracy) and \
               (robotPositionY > -1.875 - accuracy and robotPositionY < -1.875 + accuracy):
                tempZoneNumber = 1
                self.zone.SetZoneNumber(tempZoneNumber)

            elif (robotPositionX > -1.875 - accuracy and robotPositionX < -1.875 + accuracy) and\
               (robotPositionY > -2.7 - accuracy and robotPositionY < -2.7 + accuracy):
                tempZoneNumber = 2
                self.zone.SetZoneNumber(tempZoneNumber)


        elif self.zone.GetZoneColor() == ZoneColor.YELLOW:
            if (robotPositionX > 2.4 - accuracy and robotPositionX < 2.4 + accuracy) and \
               (robotPositionY > -1.875 - accuracy and robotPositionY < -1.875 + accuracy):
                tempZoneNumber = 1
                self.zone.SetZoneNumber(tempZoneNumber)

            elif (robotPositionX > 1.875 - accuracy and robotPositionX < 1.875 + accuracy) and\
               (robotPositionY > -2.7 - accuracy and robotPositionY < -2.7 + accuracy):
                tempZoneNumber = 2
                self.zone.SetZoneNumber(tempZoneNumber)

        elif self.zone.GetZoneColor() == ZoneColor.BLUE:
            if (robotPositionX > 2.4 - accuracy and robotPositionX < 2.4 + accuracy) and \
               (robotPositionY > 1.875 - accuracy and robotPositionY < 1.875 + accuracy):
                tempZoneNumber = 1
                self.zone.SetZoneNumber(tempZoneNumber)

            elif (robotPositionX > 1.875 - accuracy and robotPositionX < 1.875 + accuracy) and\
               (robotPositionY > 2.7 - accuracy and robotPositionY < 2.7 + accuracy):
                tempZoneNumber = 2
                self.zone.SetZoneNumber(tempZoneNumber)

        return tempZoneNumber

    def EscapeFromZone(self):
        if self.zone.IsInZone():
            escapePositionX, escapePositionY, escapeOrientation = self.GetZoneTagPositionAndOrientation()

            if math.isinf(escapePositionX):
                print("unknown zone color or number")
                return False

            self.MoveToPositionEscapeZone(escapePositionX, escapePositionY, escapeOrientation, 0.6, 1, 1)

            x = self.position.position.x
            y = self.position.position.y
            orientation = self.position.orientation.z

            return x, y, orientation

        return False

    def GetZoneTagPositionAndOrientation(self):

        positionX = math.inf
        positionY = math.inf
        orientation = 0

        zoneColor = self.zone.GetZoneColor()
        zoneNumber = self.zone.GetZoneNumber()

        if zoneColor == ZoneColor.BLACK:
            if zoneNumber == 1:
                positionX = -0.525
                positionY = -0.3
                orientation = math.pi

            elif zoneNumber == 2:
                positionX = -0.3
                positionY = 0.525
                orientation = math.pi / 2

            elif zoneNumber == 3:
                positionX = 0.525
                positionY = 0.3
                orientation = 0

            elif zoneNumber == 4:
                positionX = 0.3
                positionY = -0.525
                orientation = -math.pi / 2

        elif zoneColor == ZoneColor.RED:
            if zoneNumber == 1:
                positionX = -2.4
                positionY = 1.875
                orientation = -math.pi / 2

            elif zoneNumber == 2:
                positionX = -1.875
                positionY = 2.7
                orientation = 0

        elif zoneColor == ZoneColor.GREEN:
            if zoneNumber == 1:
                positionX = -2.4
                positionY = -1.875
                orientation = math.pi / 2

            elif zoneNumber == 2:
                positionX = -1.875
                positionY = -2.7
                orientation = 0

        elif zoneColor == ZoneColor.YELLOW:
            if zoneNumber == 1:
                positionX = 2.4
                positionY = -1.875
                orientation = math.pi / 2

            elif zoneNumber == 2:
                positionX = 1.875
                positionY = -2.7
                orientation = math.pi

        elif zoneColor == ZoneColor.BLUE:
            if zoneNumber == 1:
                positionX = 2.4
                positionY = 1.875
                orientation = -math.pi / 2

            elif zoneNumber == 2:
                positionX = 1.875
                positionY = 2.7
                orientation = math.pi

        return positionX, positionY, orientation

    def PutDownPuckOnPosition(self, positionX, positionY):

        if not self.CheckPositionIsEmpty(positionX, positionY):
            return False

        self.MoveToPosition(positionX, positionY, 0.05, 0.14, 1, 1)

        print("Gripper")
        self.gripper = False
        self.pubGripper.publish(self.gripper)

        if self.robotStop:
            return False
        
        return True

    def CheckPositionIsEmpty(self, positionX, positionY):
        angle, range = self.CalculateAngleAndRangeToPutDownPosition(positionX, positionY)

        puckListClone = self.puckList

        for puck in puckListClone.pucks:
            if (puck.range - range) <= 0.01 and (puck.angle - angle) <= 0.001:
                return False

        return True

    def CalculateAngleAndRangeToPutDownPosition(self, positionX, positionY):
        dX = self.position.position.x - positionX
        dY = self.position.position.y - positionY

        range = math.sqrt(dX*dX + dY*dY)

        alpha = math.atan2(dY, dX)

        angle = self.position.orientation.z - alpha
        print("dX: {}, dY: {}, alpha: {}, orientation: {}".format(dX, dY, alpha, self.position.orientation.z))
        return [angle, range]

def main():
    rospy.init_node('testBot', anonymous=True)
    if len(sys.argv) > 1:
           botNumber = sys.argv[1]
    else:
           botNumber = "1"

    print("bot{}".format(botNumber))
    TransportRobot(botNumber)
    rospy.spin()
    

if __name__ == '__main__':
    main()
