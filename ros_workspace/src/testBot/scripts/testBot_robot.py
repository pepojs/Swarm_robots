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

class OrientationFromTag(enum.IntEnum):
    NOT_SPECIFIED = -1
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3
    
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
        self.positionInZone_X = 0
        self.positionInZone_Y = 0
        self.orientationToAxisX = 0
        
    def SetZoneColor(self, color: ZoneColor):
        self.zoneColor = color

    def SetZoneNumber(self, number):
        self.zoneNumber = number

    def SetIsInZoneFlage(self, inZone: Bool):
        self.isInZone = inZone

    def SetPositionInZoneX(self, x):
        self.positionInZone_X = x

    def SetPositionInZoneY(self, y):
        self.positionInZone_Y = y

    def SetOrientationRad(self, orientation):
        self.orientationToAxisX = orientation
        
    def GetZoneColor(self):
        return self.zoneColor

    def GetZoneNumber(self):
        return self.zoneNumber

    def IsInZone(self):
        return self.isInZone

    def GetPositionInZoneX(self):
        return self.positionInZone_X

    def GetPositionInZoneY(self):
        return self.positionInZone_Y

    def GetOrientationRad(self):
        return self.orientationToAxisX
    
        
class TransportRobot:
    def __init__(self, botName):
        self.botName = botName
        self.robotStop = False
        
        self.isInZone = False
        self.zone = Zone()
        self.puckColor = PuckColor.NONPUCK 
        
        self.pubMove = rospy.Publisher("/" + botName + "/cmd_vel", Twist, queue_size=10)
        self.pubGripper = rospy.Publisher("/" + botName + "/gripper", Bool, queue_size=10)
        self.pubTurret = rospy.Publisher("/" + botName + "/turretController", Float32, queue_size=10)
        self.pubRobotAnswer = rospy.Publisher("/" + botName + "/robotAnswer", String, queue_size=10)
        self.move = Twist()
        self.gripper = Bool(False)
        self.turret = Float32(0)

        self.proximityList = ProximityList()
        self.newProximityListMsg = False
        self.baseGround = BaseGroundList().baseGrounds
        self.newBaseGroundMsg = False
        self.motoGround = MotoGroundList().motoGrounds
        self.newMotoGroundMsg = False
        self.puckList = PuckList()
        self.newPuckListMsg = False
        self.position = Position()
        self.newPositionMsg = False
        self.turretEncoder = Float32()
        self.newTurretEncoderMsg = False

        rospy.Subscriber("/" + botName + "/proximity", ProximityList, self.callbackProximity)
        rospy.Subscriber("/" + botName + "/baseGround", BaseGroundList, self.callbackBaseGround)
        rospy.Subscriber("/" + botName + "/motoGround", MotoGroundList, self.callbackMotoGround)
        rospy.Subscriber("/" + botName + "/puck_list", PuckList, self.callbackPuckList)
        rospy.Subscriber("/" + botName + "/position", Position, self.callbackPosition)
        rospy.Subscriber("/" + botName + "/robotStop", Bool, self.callbackRobotStop)
        rospy.Subscriber("/" + botName + "/turretEncoder", Float32, self.callbackTurretEncoder)
        rospy.Subscriber("/" + botName + "/robotController", String, self.callbackRobotController)
        
        while self.newMotoGroundMsg == False:
            pass

        #self.TakeNearestPuck()
        #self.EnterToZone(ZoneColor.BLACK)
        #self.zone.SetZoneNumber(1)
        #self.zone.SetIsInZoneFlage(True)
        #self.GetPositionAndOrientationInZone()
        '''
        self.EnterToZone(ZoneColor.BLACK)
        self.TakeNearestPuck()
        self.EscapeFromZone()

        self.MoveToForward()
        self.RotateLeft()
        self.MoveToForward()
        self.MoveToForward()
        self.MoveToForward()
        self.MoveToForward()
        self.RotateRight()
        self.MoveToForward()
        
        self.EnterToZone(ZoneColor.RED)
        self.PutDownPuckOnPosition(-0.1, -0.1)
        self.EscapeFromZone()
        '''

    def callbackProximity(self, msg):
        #self.move.linear.x = 0.5
        #if msg.proximities[0].value > 0:
        #    self.move.linear.x = 0

        #print(msg.proximities[0])
        #self.pubMove.publish(self.move)

        self.proximityList = msg
        self.newProximityListMsg = True
        
    def callbackBaseGround(self, msg):
        self.baseGround = msg.baseGrounds
        self.newBaseGroundMsg = True

    def callbackMotoGround(self, msg):
        self.motoGround =msg.motoGrounds
        self.newMotoGroundMsg = True

    def callbackPuckList(self, msg):
        self.puckList = msg
        self.newPuckListMsg = True

    def callbackPosition(self, msg):
        self.position = msg
        self.newPositionMsg = True

    def callbackRobotStop(self, msg):
        self.robotStop = msg.data

    def callbackTurretEncoder(self, msg):
        self.turretEncoder = msg
        self.newTurretEncoderMsg = True

    def callbackRobotController(self, msg):
        command, paramList = self.decode_command(msg.data)
        print(msg)
        answer = 'not found'
        answerParam = []
        if command == 'forward':
            answer = 'forward'
            answerParam.append(self.MoveToForward())

        elif command == 'rotateLeft':
            answer = 'rotateLeft'
            answerParam.append(self.RotateLeft())

        elif command == 'rotateRight':
            answer = 'rotateRight'
            answerParam.append(self.RotateRight())

        elif command == 'enterToZone' and len(paramList) == 1:
            answer = 'enterToZone'
            answerParam.append(self.EnterToZone(paramList[0]))

        elif command == 'takeNearestPuck':
            answer = 'takeNearestPuck'
            answerParam.append(self.TakeNearestPuck())

        elif command == 'escapeFromZone':
            answer = 'escapeFromZone'
            answerParam.append(self.EscapeFromZone())

        elif command == 'putDownPuckOnPosition' and len(paramList) == 2:
            answer = 'putDownPuckOnPosition'
            answerParam.append(self.PutDownPuckOnPosition(paramList[0], paramList[1]))

        elif command == 'getPuckColor':
            answer = 'getPuckColor'
            answerParam.append(self.GetPuckColor())

        elif command == 'getOrientationFromTag':
            answer = 'getOrientationFromTag'
            answerParam.append(self.GetOrientationFromTag())

        elif command == 'moveToPosition' and len(paramList) == 2:
            answer = 'moveToPosition'
            answerParam.append(self.MoveToPosition(paramList[0], paramList[1]))

        self.pubRobotAnswer.publish(self.code_command(answer, answerParam))



    def decode_command(self, sentence):
        command = json.loads(sentence)
        return command['name'], command['values']

    def code_command(self, command_name, paramList):
        value = []
        for item in paramList:
            value.append(item)
        code = json.dumps({"name": command_name, "values": value})
        return code

    def GetPuckColor(self):
        return self.puckColor
        
    def GetOrientationFromTag(self):
        if (self.motoGround[0].value > 0.49 and self.motoGround[0].value < 0.51) and \
           (self.motoGround[3].value > 0.44 and self.motoGround[3].value < 0.46):
            return OrientationFromTag.EAST

        elif (self.motoGround[0].value > 0.44 and self.motoGround[0].value < 0.46) and \
             (self.motoGround[3].value > 0.34 and self.motoGround[3].value < 0.385):
            return OrientationFromTag.SOUTH

        elif (self.motoGround[0].value > 0.34 and self.motoGround[0].value < 0.385) and \
             (self.motoGround[3].value > 0.39 and self.motoGround[3].value < 0.41):
            return OrientationFromTag.WEST

        elif (self.motoGround[0].value > 0.39 and self.motoGround[0].value < 0.41) and \
             (self.motoGround[3].value > 0.49 and self.motoGround[3].value < 0.51):
            return OrientationFromTag.NORTH

        else:
            return OrientationFromTag.NOT_SPECIFIED

    def MoveToPosition(self, x, y):

        run = True
        while run and not self.robotStop:
            #if self.newPositionMsg:
            #    self.newPositionMsg = False

            [angle, range] = self.CalculateAngleAndRangeToPosition(x, y)

            if range < 0.001:
                self.move.linear.x = 0
                self.move.angular.z = 0
                run = False

            elif range > 0.05:
                if angle > 0.1:
                    self.move.linear.x = 0
                    self.move.angular.z = -2.5

                elif angle < -0.1:
                    self.move.linear.x = 0
                    self.move.angular.z = 2.5

                elif angle > 0.03:
                    self.move.linear.x = 0
                    self.move.angular.z = -0.5

                elif angle < -0.03:
                    self.move.linear.x = 0
                    self.move.angular.z = 0.5

                elif angle > 0.01:
                    self.move.linear.x = 0
                    self.move.angular.z = -0.1

                elif angle < -0.01:
                    self.move.linear.x = 0
                    self.move.angular.z = 0.1

                elif range >= 0.05:
                    self.move.linear.x = 2
                    self.move.angular.z = 0

            elif range <= 0.05:

                if angle < 0.01 and angle > -0.01 and range >= 0.01:
                    self.move.linear.x = 0.1
                    self.move.angular.z = 0

                elif angle < 0.01 and angle > -0.01 and range >= 0.001:
                    self.move.linear.x = 0.05
                    self.move.angular.z = 0

                elif angle > math.pi-0.01 or angle < -math.pi+0.01 and range >= 0.01:
                    self.move.linear.x = -0.1
                    self.move.angular.z = 0

                elif angle > math.pi-0.01 or angle < -math.pi+0.01 and range >= 0.001:
                    self.move.linear.x = -0.05
                    self.move.angular.z = 0

                elif angle >= 0.01 and angle < 0.03:
                    self.move.linear.x = 0
                    self.move.angular.z = -0.1

                elif angle < -0.01 and angle > -0.03:
                    self.move.linear.x = 0
                    self.move.angular.z = 0.1

                elif angle >= 0.03 and angle < 0.1:
                    self.move.linear.x = 0
                    self.move.angular.z = -0.5

                elif angle <= -0.03 and angle > -0.1:
                    self.move.linear.x = 0
                    self.move.angular.z = 0.5

                elif angle >= 0.1 and angle < math.pi/2:
                    self.move.linear.x = 0
                    self.move.angular.z = -2.5

                elif angle <= -0.1 and angle > -math.pi/2:
                    self.move.linear.x = 0
                    self.move.angular.z = 2.5

                elif angle <= math.pi-0.01 and angle >= math.pi-0.03: # or angle < -math.pi-0.01 and angle >= -math.pi-0.03:
                    self.move.linear.x = 0
                    self.move.angular.z = -0.1

                elif angle >= -math.pi+0.01 and angle <= -math.pi+0.03:
                    self.move.linear.x = 0
                    self.move.angular.z = 0.1

                elif angle <= math.pi-0.03 and angle > math.pi-0.1: # or angle < -math.pi-0.03 and angle >= -math.pi-0.1:
                    self.move.linear.x = 0
                    self.move.angular.z = -0.5

                elif angle >= -math.pi+0.03 and angle < -math.pi+0.1:
                    self.move.linear.x = 0
                    self.move.angular.z = 0.5

                elif angle <= math.pi-0.1 and angle >= math.pi/2: # or angle < -math.pi-0.1 and angle >= -3*math.pi/2:
                    self.move.linear.x = 0
                    self.move.angular.z = -2.5

                elif angle >= -math.pi+0.1 and angle <= -math.pi/2:
                    self.move.linear.x = 0
                    self.move.angular.z = 2.5



            self.pubMove.publish(self.move)


        return self.GetOrientationFromTag()

    def CalculateAngleAndRangeToPosition(self, x, y):
        robotPosition = self.position.position
        dX = x - robotPosition.x
        dY = y - robotPosition.y

        range = math.sqrt(dX * dX + dY * dY)

        alpha = math.atan2(dY, dX)
        robotOrientation = self.position.orientation.z
        dTheta = robotOrientation - alpha

        # if dTheta is more then 180 we won't make rotation 270 but 90 in opposite size
        if dTheta < -math.pi-0.3:
            dTheta = -dTheta - math.pi

        elif dTheta > math.pi+0.3:
            dTheta = -dTheta + math.pi

        print('alpha: {}, orie: {}, dtheta: {}'.format(alpha, robotOrientation, dTheta))
        print('range: {}'.format(range))


        return [dTheta, range]


    def MoveToForward(self):
        self.move.linear.x = 0.1
            
        if abs(self.motoGround[0].value - self.motoGround[1].value) > 0.01 and \
           abs(self.motoGround[1].value - self.motoGround[2].value) > 0.01 and \
           abs(self.motoGround[2].value - self.motoGround[3].value) > 0.01 and \
           abs(self.motoGround[3].value - self.motoGround[0].value) > 0.01 :

            #jezeli baseGround 0 i 4 nie sa rowne 0 to poprawic pozycje - dopisac !
            
            run = True
            while run and not self.robotStop:
                if self.newMotoGroundMsg == True:
                    self.newMotoGroundMsg = False
            
                    if (self.motoGround[0].value > 0.51 or self.motoGround[0].value < 0.34) and \
                       (self.motoGround[3].value > 0.51 or self.motoGround[3].value < 0.34):
                        run = False
                        self.move.linear.x = 0
                        self.pubMove.publish(self.move)
                        break
                    else:
                        self.move.linear.x = 0.1
                        self.pubMove.publish(self.move)

        run = True
        while run and not self.robotStop:
            if self.newMotoGroundMsg == True:
                self.newMotoGroundMsg = False

                if self.motoGround[0].value <= 0.01 and self.motoGround[3].value <= 0.01:
                    self.move.linear.x = 0.3
                    self.move.angular.z = 0
                    
                elif self.motoGround[0].value >= 0.99 and self.motoGround[3].value <= 0.01:
                    self.move.linear.x = 0.05
                    self.move.angular.z = -0.1
                                        
                elif self.motoGround[0].value <= 0.01 and self.motoGround[3].value >= 0.99:
                    self.move.linear.x = 0.05
                    self.move.angular.z = 0.1
                    
                elif (self.motoGround[0].value >= 0.34 and self.motoGround[0].value <= 0.51) and\
                     (self.motoGround[3].value <= 0.01 or self.motoGround[3].value >= 0.99):
                    self.move.linear.x = 0.05
                    self.move.angular.z = 0.1
                                        
                elif (self.motoGround[3].value >= 0.34 and self.motoGround[3].value <= 0.51) and\
                     (self.motoGround[0].value <= 0.01 or self.motoGround[0].value >= 0.99):
                    self.move.linear.x = 0.05
                    self.move.angular.z = -0.1
                                        
                elif (self.motoGround[0].value >= 0.34 and self.motoGround[0].value <= 0.51) and\
                     (self.motoGround[3].value >= 0.34 and self.motoGround[3].value <= 0.51):
                    run = False
                    self.move.linear.x = 0
                    self.move.angular.z = 0

                elif self.motoGround[0].value >= 0.99 and self.motoGround[3].value >= 0.99:
                    print('Robot: {} got lost'.format(self.botName))
                    run = False
                    return False

                self.pubMove.publish(self.move)

        run = True
        while run and not self.robotStop:
            if self.newMotoGroundMsg == True:
                self.newMotoGroundMsg = False

                if (self.motoGround[1].value <= 0.01 and self.motoGround[2].value <= 0.01):
                    self.move.linear.x = 0.05
                    self.move.angular.z = 0

                elif self.motoGround[1].value >= 0.99 and self.motoGround[2].value <= 0.01:
                    self.move.linear.x = 0.05
                    self.move.angular.z = 0.1

                elif self.motoGround[1].value <= 0.01 and self.motoGround[2].value >= 0.99:
                    self.move.linear.x = 0.05
                    self.move.angular.z = -0.1

                elif abs(self.motoGround[0].value - self.motoGround[1].value) > 0.01 and \
                     abs(self.motoGround[1].value - self.motoGround[2].value) > 0.01 and \
                     abs(self.motoGround[2].value - self.motoGround[3].value) > 0.01 and \
                     abs(self.motoGround[3].value - self.motoGround[0].value) > 0.01:

                    self.move.linear.x = 0.05
                    self.move.angular.z = 0
                    
                    if self.baseGround[1].value >= 0.99 and self.baseGround[3].value >= 0.99 and \
                       self.baseGround[5].value >= 0.99 and self.baseGround[7].value >= 0.99:

                        self.move.linear.x = 0.0
                        run = False
                    
                self.pubMove.publish(self.move)

        return True

    def RotateLeft(self):
        if abs(self.motoGround[0].value - self.motoGround[1].value) > 0.01 and \
           abs(self.motoGround[1].value - self.motoGround[2].value) > 0.01 and \
           abs(self.motoGround[2].value - self.motoGround[3].value) > 0.01 and \
           abs(self.motoGround[3].value - self.motoGround[0].value) > 0.01:

            orientationTag = self.GetOrientationFromTag()

            if orientationTag == OrientationFromTag.NORTH:
                finishOrientationTag = OrientationFromTag.WEST
            else:
                finishOrientationTag = orientationTag - 1

            if finishOrientationTag == OrientationFromTag.NORTH:
                finishOrientation = 0

            elif finishOrientationTag == OrientationFromTag.EAST:
                finishOrientation = -math.pi/2

            elif finishOrientationTag == OrientationFromTag.SOUTH:
                finishOrientation = math.pi

            elif finishOrientationTag == OrientationFromTag.WEST:
                finishOrientation = math.pi/2

                      
        run = True
        while run and not self.robotStop:
            if self.newMotoGroundMsg == True:
                self.newMotoGroundMsg = False
                #print('finishOrientationTag: {}, orientation tag: {}'.format(finishOrientationTag, self.GetOrientationFromTag()))
                if finishOrientationTag != self.GetOrientationFromTag():
                    self.move.linear.x = 0
                    self.move.angular.z = 0.8
                
                elif abs(self.motoGround[0].value - self.motoGround[1].value) <= 0.015 or \
                     abs(self.motoGround[1].value - self.motoGround[2].value) <= 0.015 or \
                     abs(self.motoGround[2].value - self.motoGround[3].value) <= 0.015 or \
                     abs(self.motoGround[3].value - self.motoGround[0].value) <= 0.015:
                    self.move.linear.x = 0
                    self.move.angular.z = 0.6

                elif finishOrientationTag == self.GetOrientationFromTag():

                    if finishOrientation - self.position.orientation.z > 0.0116 and \
                       finishOrientation - self.position.orientation.z <= 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.05

                    elif finishOrientationTag == OrientationFromTag.SOUTH and \
                         finishOrientation - self.position.orientation.z < 2*math.pi - 0.0116 and \
                         finishOrientation - self.position.orientation.z >= 2*math.pi - 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.05

                    elif finishOrientationTag != OrientationFromTag.SOUTH and \
                         finishOrientation - self.position.orientation.z < -0.0116 and\
                         finishOrientation - self.position.orientation.z >= -0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.05

                    elif (finishOrientationTag != OrientationFromTag.SOUTH and
                          finishOrientation - self.position.orientation.z < 0.012 and
                          finishOrientation - self.position.orientation.z > -0.012) or\
                         (finishOrientationTag == OrientationFromTag.SOUTH and
                          finishOrientation - self.position.orientation.z < 0.012 or
                          finishOrientation - self.position.orientation.z > 2*math.pi - 0.012):
                        self.move.linear.x = 0
                        self.move.angular.z = 0.0
                        self.pubMove.publish(self.move)
                        run = False
                        break
                        
                    else:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.15

                self.pubMove.publish(self.move)

        return self.GetOrientationFromTag()

    def RotateRight(self):
        if abs(self.motoGround[0].value - self.motoGround[1].value) > 0.01 and \
           abs(self.motoGround[1].value - self.motoGround[2].value) > 0.01 and \
           abs(self.motoGround[2].value - self.motoGround[3].value) > 0.01 and \
           abs(self.motoGround[3].value - self.motoGround[0].value) > 0.01 :

            orientationTag = self.GetOrientationFromTag()

            if orientationTag == OrientationFromTag.WEST:
                finishOrientationTag = OrientationFromTag.NORTH
            else:
                finishOrientationTag = orientationTag + 1

            if finishOrientationTag == OrientationFromTag.NORTH:
                finishOrientation = 0
            elif finishOrientationTag == OrientationFromTag.EAST:
                finishOrientation = -math.pi/2

            elif finishOrientationTag == OrientationFromTag.SOUTH:
                finishOrientation = math.pi

            elif finishOrientationTag == OrientationFromTag.WEST:
                finishOrientation = math.pi/2
            
        run = True
        while run and not self.robotStop:
            if self.newMotoGroundMsg == True:
                self.newMotoGroundMsg = False

                if finishOrientationTag != self.GetOrientationFromTag():
                    self.move.linear.x = 0
                    self.move.angular.z = -0.8
                
                elif abs(self.motoGround[0].value - self.motoGround[1].value) <= 0.015 or \
                     abs(self.motoGround[1].value - self.motoGround[2].value) <= 0.015 or \
                     abs(self.motoGround[2].value - self.motoGround[3].value) <= 0.015 or \
                     abs(self.motoGround[3].value - self.motoGround[0].value) <= 0.015:
                    self.move.linear.x = 0
                    self.move.angular.z = -0.6

                elif finishOrientationTag == self.GetOrientationFromTag():

                    if finishOrientation - self.position.orientation.z > 0.0116 and \
                            finishOrientation - self.position.orientation.z <= 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.05

                    elif finishOrientationTag == OrientationFromTag.SOUTH and \
                            finishOrientation - self.position.orientation.z < 2 * math.pi - 0.0116 and \
                            finishOrientation - self.position.orientation.z >= 2 * math.pi - 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.05

                    elif finishOrientationTag != OrientationFromTag.SOUTH and \
                            finishOrientation - self.position.orientation.z < -0.0116 and \
                            finishOrientation - self.position.orientation.z >= -0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.05

                    elif (finishOrientationTag != OrientationFromTag.SOUTH and
                          finishOrientation - self.position.orientation.z < 0.012 and
                          finishOrientation - self.position.orientation.z > -0.012) or\
                         (finishOrientationTag == OrientationFromTag.SOUTH and
                          finishOrientation - self.position.orientation.z < 0.012 or
                          finishOrientation - self.position.orientation.z > 2*math.pi - 0.012):
                        self.move.linear.x = 0
                        self.move.angular.z = 0.0
                        self.pubMove.publish(self.move)
                        run = False
                        break
                        
                    else:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.15

            
                self.pubMove.publish(self.move)

        return self.GetOrientationFromTag()

    def TakeNearestPuck(self):            
        [puckPositionX, puckPositionY] = self.CalculateNearestPuckPosition()

        if puckPositionX == 1 and puckPositionY == 1:
            self.move.linear.x = 0.3
            self.move.angular.z = 0
            self.pubMove.publish(self.move)
            rospy.sleep(1)
            self.move.linear.x = 0
            self.move.angular.z = 0
            self.pubMove.publish(self.move)
            return PuckColor.NONPUCK

        [puckAngle, puckRange] = self.CalculatePuckAngleAndRange(puckPositionX, puckPositionY)
        #print("puck angle: ", puckAngle)
        #print("puck range: ", puckRange)
        run = True
        while run and not self.robotStop:
            [puckAngle, puckRange] = self.CalculatePuckAngleAndRange(puckPositionX, puckPositionY)

                 
            if puckRange <= 0.14:
                self.move.linear.x = 0
                self.move.angular.z = 0
                run = False
                            
            elif puckAngle >= 0.05:
                self.move.linear.x = 0
                self.move.angular.z = 0.4

            elif puckAngle <= -0.05:
                self.move.linear.x = 0
                self.move.angular.z = -0.4
                
            elif puckAngle >= 0.001:
                self.move.linear.x = 0
                self.move.angular.z = 0.01
                            
            elif puckAngle <= -0.001:
                self.move.linear.x = 0
                self.move.angular.z = -0.01

            elif puckRange >= 0.05:
                self.move.linear.x = 0.2
                self.move.angular.z = 0

            elif puckRange >= 0.001:
                self.move.linear.x = 0.01
                self.move.angular.z = 0

                
            self.pubMove.publish(self.move)

        self.gripper = True
        self.pubGripper.publish(self.gripper)
        return self.GetPuckColor()

    def CalculateNearestPuckPosition(self):
        self.GetPositionAndOrientationInZone()
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
                alpha = puck.angle - self.zone.GetOrientationRad()

                dX = math.cos(alpha) * puck.range*0.01
                dY = math.sin(alpha) * puck.range*0.01

                #print("dX: {}, dY: {}".format(dX, dY))
                if(self.zone.GetPositionInZoneX() + dX) < 0 and (self.zone.GetPositionInZoneY() + dY) < 0:
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

        self.SetPuckColor(puckInZoneList[nearestPuck].color)
         
        puckAngle = puckInZoneList[nearestPuck].angle
        puckRange = puckInZoneList[nearestPuck].range*0.01

        alpha = puckAngle - self.zone.GetOrientationRad()

        dX = math.cos(alpha)*puckRange
        dY = math.sin(alpha)*puckRange

        puckPositionX = self.zone.GetPositionInZoneX() + dX
        puckPositionY = self.zone.GetPositionInZoneY() + dY
        
        return [puckPositionX, puckPositionY]

    def SetPuckColor(self, color):
        if color.r == 255 and color.g == 0 and color.b == 0:
            self.puckColor = PuckColor.RED
        elif color.r == 0 and color.g == 255 and color.b == 0:
            self.puckColor = PuckColor.GREEN
        elif color.r == 0 and color.g == 0 and color.b == 255:
            self.PuckColor = PuckColor.BLUE
        elif color.r == 255 and color.g == 255 and color.b == 0:
            self.PuckColor = PuckColor.YELLOW
        else:
            self.PuckColor = PuckColor.UNRECOGNIZED

    def CalculatePuckAngleAndRange(self, puckPositionX, puckPositionY):
        self.GetPositionAndOrientationInZone()
        
        dX = self.zone.GetPositionInZoneX() - puckPositionX
        dY = self.zone.GetPositionInZoneY() - puckPositionY
        
        range = math.sqrt(dX*dX + dY*dY)

        alpha = math.asin(dY/range)

        puckAngle = self.zone.GetOrientationRad() - alpha

        return [puckAngle, range]

    def EnterToZone(self, zoneColor: ZoneColor):
        self.zone.SetZoneColor(zoneColor)
        self.zone.SetIsInZoneFlage(True)
        tempZoneNumber = -1
        if zoneColor == ZoneColor.BLACK:
            
            if (self.motoGround[0].value >= 0.39 and self.motoGround[0].value <= 0.41) and \
               (self.motoGround[3].value >= 0.49 and self.motoGround[3].value <= 0.51):

                tempZoneNumber = 1
                self.zone.SetZoneNumber(1)
                self.GetPositionAndOrientationInZone()

            elif (self.motoGround[0].value >= 0.49 and self.motoGround[0].value <= 0.51) and \
                 (self.motoGround[3].value >= 0.44 and self.motoGround[3].value <= 0.46):

                tempZoneNumber = 2
                self.zone.SetZoneNumber(2)
                self.GetPositionAndOrientationInZone()

            elif (self.motoGround[0].value >= 0.44 and self.motoGround[0].value <= 0.46) and \
                 (self.motoGround[3].value >= 0.34 and self.motoGround[3].value <= 0.36):

                tempZoneNumber = 3
                self.zone.SetZoneNumber(3)
                self.GetPositionAndOrientationInZone()

            elif (self.motoGround[0].value >= 0.34 and self.motoGround[0].value <= 0.36) and \
                 (self.motoGround[3].value >= 0.39 and self.motoGround[3].value <= 0.41):

                tempZoneNumber = 4
                self.zone.SetZoneNumber(4)
                self.GetPositionAndOrientationInZone()

        if zoneColor == ZoneColor.RED:
            if(self.motoGround[0].value >= 0.34 and self.motoGround[0].value <= 0.36) and \
              (self.motoGround[3].value >= 0.39 and self.motoGround[3].value <= 0.41):

                tempZoneNumber = 1
                self.zone.SetZoneNumber(1)
                self.GetPositionAndOrientationInZone()

            elif (self.motoGround[0].value >= 0.44 and self.motoGround[0].value <= 0.46) and \
                 (self.motoGround[3].value >= 0.34 and self.motoGround[3].value <= 0.36):

                tempZoneNumber = 2
                self.zone.SetZoneNumber(2)
                self.GetPositionAndOrientationInZone()

        #print(zoneColor)
        #print(self.zone.GetZoneNumber())
        return tempZoneNumber

    def GetPositionAndOrientationInZone(self):
        if self.zone.IsInZone():

            if self.zone.GetZoneColor() == ZoneColor.BLACK:
                if self.zone.GetZoneNumber() == 1:
                    [centerLanternRange, centerLanternAngle, secondLanternRange, secondLanternAngle] \
                        = self.GetOrangeBrownLantern()

                elif self.zone.GetZoneNumber() == 2:
                    [centerLanternRange, centerLanternAngle, secondLanternRange, secondLanternAngle] \
                        = self.GetOrangeCyanLantern()

                elif self.zone.GetZoneNumber() == 3:
                    [centerLanternRange, centerLanternAngle, secondLanternRange, secondLanternAngle] \
                        = self.GetOrangeMagentaLantern()

                elif self.zone.GetZoneNumber() == 4:
                    [centerLanternRange, centerLanternAngle, secondLanternRange, secondLanternAngle] \
                        = self.GetOrangePurpleLantern()
                else:
                    print("Zone number is not correct")
                    return

                if centerLanternRange == -1 or secondLanternRange == -1:
                    print("Can't get position in zone")
                    return
                if centerLanternRange == -2 and secondLanternRange == -2:
                    # print("Not new position")
                    return

                centerSecondRange = 0.525

                alpha = math.acos((centerSecondRange * centerSecondRange \
                                   + centerLanternRange * centerLanternRange \
                                   - secondLanternRange * secondLanternRange) \
                                  / (2 * centerSecondRange * centerLanternRange))

                self.zone.SetPositionInZoneY(-math.sin(alpha) * centerLanternRange)
                self.zone.SetPositionInZoneX(-math.cos(alpha) * centerLanternRange)
                #print("position in zone x: ", self.zone.GetPositionInZoneX())
                #print("position in zone y: ", self.zone.GetPositionInZoneY())

                beta = math.asin(abs(self.zone.GetPositionInZoneY()) / centerLanternRange)
                if centerLanternAngle <= (-math.pi + beta) and centerLanternAngle >= -math.pi:
                    self.zone.SetOrientationRad((2 * math.pi + centerLanternAngle - beta))
                else:
                    self.zone.SetOrientationRad((centerLanternAngle - beta))

                #self.zone.SetOrientationRad(centerLanternAngle - beta)
                #print("orientation to axis x: ", self.zone.GetOrientationRad())

            elif self.zone.GetZoneColor() == ZoneColor.RED:
                if self.zone.GetZoneNumber() == 1:
                    [centerLanternRange, centerLanternAngle, secondLanternRange,
                     secondLanternAngle] = self.GetOrangeBrownLantern()
                    centerSecondRange = 0.55

                    [centerLanternRange2, centerLanternAngle2, secondLanternRange2,
                     secondLanternAngle2] = self.GetBrownPurpleLantern()

                    if (centerLanternRange == -1 or secondLanternRange == -1) or\
                        ((centerLanternRange2 != -1 and secondLanternRange2 != -1) and centerLanternRange > secondLanternRange2):
                        [centerLanternRange, centerLanternAngle, secondLanternRange,
                         secondLanternAngle] = [centerLanternRange2, centerLanternAngle2, secondLanternRange2,
                     secondLanternAngle2]
                        centerSecondRange = 0.525


                elif self.zone.GetZoneNumber() == 2:

                    [centerLanternRange, centerLanternAngle, secondLanternRange,
                     secondLanternAngle] = self.GetOrangeBrownLantern()
                    centerSecondRange = 0.55

                    if centerLanternRange == -1 or secondLanternRange == -1:
                        [centerLanternRange, centerLanternAngle, secondLanternRange,
                         secondLanternAngle] = self.GetBrownPurpleLantern()
                        centerSecondRange = 0.525

                else:
                    print("Zone number is not correct")
                    return

                if centerLanternRange == -1 or secondLanternRange == -1:
                    print("Can't get position in zone")
                    return
                if centerLanternRange == -2 and secondLanternRange == -2:
                    # print("Not new position")
                    return

                if self.zone.GetZoneNumber() == 1:
                    constRangeLanternCenterAndMainCenter = 0
                    if centerSecondRange == 0.525:
                        constRangeLanternCenterAndMainCenter = -0.55

                    print(((centerSecondRange * centerSecondRange \
                            + centerLanternRange * centerLanternRange \
                            - secondLanternRange * secondLanternRange) \
                           / (2 * centerSecondRange * centerLanternRange)))
                    print(centerLanternRange)
                    alpha = math.acos((centerSecondRange * centerSecondRange \
                                       + centerLanternRange * centerLanternRange \
                                       - secondLanternRange * secondLanternRange) \
                                      / (2 * centerSecondRange * centerLanternRange))

                    self.zone.SetPositionInZoneY(-math.sin(alpha) * centerLanternRange)
                    self.zone.SetPositionInZoneX(constRangeLanternCenterAndMainCenter - math.cos(alpha) * centerLanternRange)
                    print("position in zone x: ", self.zone.GetPositionInZoneX())
                    print("position in zone y: ", self.zone.GetPositionInZoneY())

                    if centerSecondRange == 0.525:
                        beta = math.acos(abs(self.zone.GetPositionInZoneY()) / secondLanternRange)
                        if secondLanternAngle >= (math.pi - beta) and secondLanternAngle <= math.pi:
                            self.zone.SetOrientationRad((-2*math.pi + secondLanternAngle + beta))
                        else:
                            self.zone.SetOrientationRad((secondLanternAngle + beta))

                    else:
                        beta = math.acos(abs(self.zone.GetPositionInZoneY()) / centerLanternRange)
                        self.zone.SetOrientationRad((centerLanternAngle - beta))

                    print("orientation to axis x: ", self.zone.GetOrientationRad())


                elif self.zone.GetZoneNumber() == 2:
                    constRangeLanternCenterAndMainCenter = 0
                    if centerSecondRange == 0.525:
                        constRangeLanternCenterAndMainCenter = -0.55

                    print(((centerSecondRange * centerSecondRange \
                                       + centerLanternRange * centerLanternRange \
                                       - secondLanternRange * secondLanternRange) \
                                      / (2 * centerSecondRange * centerLanternRange)))
                    alpha = math.acos((centerSecondRange * centerSecondRange \
                                       + centerLanternRange * centerLanternRange \
                                       - secondLanternRange * secondLanternRange) \
                                      / (2 * centerSecondRange * centerLanternRange))

                    self.zone.SetPositionInZoneY(-math.sin(alpha) * centerLanternRange)
                    self.zone.SetPositionInZoneX(constRangeLanternCenterAndMainCenter-math.cos(alpha) * centerLanternRange)
                    print("position in zone x: ", self.zone.GetPositionInZoneX())
                    print("position in zone y: ", self.zone.GetPositionInZoneY())

                    beta = math.asin(abs(self.zone.GetPositionInZoneY()) / centerLanternRange)
                    self.zone.SetOrientationRad(centerLanternAngle - beta)
                    print("orientation to axis x: ", self.zone.GetOrientationRad())


    def GetOrangeBrownLantern(self):
        centerLanternRange = -2
        centerLanternAngle = 0

        secondLanternRange = -2
        secondLanternAngle = 0

        if self.newPuckListMsg == True:
            self.newPuckListMsg = False

            centerLanternRange = -1
            centerLanternAngle = 0

            secondLanternRange = -1
            secondLanternAngle = 0

            puckListClone = self.puckList

            for puck in puckListClone.pucks:

                if centerLanternRange > -1 and secondLanternRange > -1:
                    break

                if puck.color.r == 255 and puck.color.g == 140 and puck.color.b == 0:
                    centerLanternRange = puck.range/100
                    centerLanternAngle = puck.angle

                elif puck.color.r == 165 and puck.color.g == 42 and puck.color.b == 42:
                    secondLanternRange = puck.range/100
                    secondLanternAngle = puck.angle

        return [centerLanternRange, centerLanternAngle, secondLanternRange, secondLanternAngle]

    def GetOrangePurpleLantern(self):
        centerLanternRange = -2
        centerLanternAngle = 0

        secondLanternRange = -2
        secondLanternAngle = 0

        if self.newPuckListMsg == True:
            self.newPuckListMsg = False

            centerLanternRange = -1
            centerLanternAngle = 0

            secondLanternRange = -1
            secondLanternAngle = 0

            puckListClone = self.puckList

            for puck in puckListClone.pucks:

                if centerLanternRange > -1 and secondLanternRange > -1:
                    break

                if puck.color.r == 255 and puck.color.g == 140 and puck.color.b == 0:
                    centerLanternRange = puck.range / 100
                    centerLanternAngle = puck.angle

                elif puck.color.r == 160 and puck.color.g == 32 and puck.color.b == 240:
                    secondLanternRange = puck.range / 100
                    secondLanternAngle = puck.angle

        return [centerLanternRange, centerLanternAngle, secondLanternRange, secondLanternAngle]

    def GetOrangeMagentaLantern(self):
        centerLanternRange = -2
        centerLanternAngle = 0

        secondLanternRange = -2
        secondLanternAngle = 0

        if self.newPuckListMsg == True:
            self.newPuckListMsg = False

            centerLanternRange = -1
            centerLanternAngle = 0

            secondLanternRange = -1
            secondLanternAngle = 0

            puckListClone = self.puckList

            for puck in puckListClone.pucks:

                if centerLanternRange > -1 and secondLanternRange > -1:
                    break

                if puck.color.r == 255 and puck.color.g == 140 and puck.color.b == 0:
                    centerLanternRange = puck.range / 100
                    centerLanternAngle = puck.angle

                elif puck.color.r == 255 and puck.color.g == 0 and puck.color.b == 255:
                    secondLanternRange = puck.range / 100
                    secondLanternAngle = puck.angle

        return [centerLanternRange, centerLanternAngle, secondLanternRange, secondLanternAngle]

    def GetOrangeCyanLantern(self):
        centerLanternRange = -2
        centerLanternAngle = 0

        secondLanternRange = -2
        secondLanternAngle = 0

        if self.newPuckListMsg == True:
            self.newPuckListMsg = False

            centerLanternRange = -1
            centerLanternAngle = 0

            secondLanternRange = -1
            secondLanternAngle = 0

            puckListClone = self.puckList

            for puck in puckListClone.pucks:

                if centerLanternRange > -1 and secondLanternRange > -1:
                    break

                if puck.color.r == 255 and puck.color.g == 140 and puck.color.b == 0:
                    centerLanternRange = puck.range / 100
                    centerLanternAngle = puck.angle

                elif puck.color.r == 0 and puck.color.g == 255 and puck.color.b == 255:
                    secondLanternRange = puck.range / 100
                    secondLanternAngle = puck.angle

        return [centerLanternRange, centerLanternAngle, secondLanternRange, secondLanternAngle]

    def GetBrownPurpleLantern(self):

        centerLanternRange = -1
        centerLanternAngle = 0

        secondLanternRange = -1
        secondLanternAngle = 0

        puckListClone = self.puckList

        for puck in puckListClone.pucks:

            if centerLanternRange > -1 and secondLanternRange > -1:
                break

            if puck.color.r == 165 and puck.color.g == 42 and puck.color.b == 42:
                centerLanternRange = puck.range / 100
                centerLanternAngle = puck.angle

            elif puck.color.r == 160 and puck.color.g == 32 and puck.color.b == 240:
                secondLanternRange = puck.range / 100
                secondLanternAngle = puck.angle

        return [centerLanternRange, centerLanternAngle, secondLanternRange, secondLanternAngle]

    def EscapeFromZone(self):
        if self.zone.IsInZone():
        
            self.GetPositionAndOrientationInZone()
            if self.zone.GetZoneColor() == ZoneColor.BLACK:
                escapePositionX = -0.525
                escapePositionY = -0.3

                [escapeAngle, escapeRange] = self.CalculateEscapeAngleAndRange(escapePositionX, escapePositionY)

                #print("escape angle: ", escapeAngle)
                #print("escepe range: ", escapeRange)
                run = True
                while run and not self.robotStop:
                    [escapeAngle, escapeRange] = self.CalculateEscapeAngleAndRange(escapePositionX, escapePositionY)

                    if escapeRange <= 0.0001:
                        self.move.linear.x = 0
                        self.move.angular.z = 0
                        run = False

                    elif escapeAngle >= 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = 1

                    elif escapeAngle <= -0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = -1

                    elif escapeAngle >= 0.001:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.05

                    elif escapeAngle <= -0.001:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.05

                    elif escapeRange >= 0.05:
                        self.move.linear.x = -0.5
                        self.move.angular.z = 0

                    elif escapeRange >= 0.001:
                        self.move.linear.x = -0.05
                        self.move.angular.z = 0

                    elif escapeRange > 0.0001:
                        self.move.linear.x = -0.005
                        self.move.angular.z = 0

                    self.pubMove.publish(self.move)

                run = True
                while run and not self.robotStop:
                    self.GetPositionAndOrientationInZone()
                    temp = self.zone.GetOrientationRad()

                    if temp >= 0 and temp <= math.pi - 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = -2

                    elif temp >= 0 and temp <= math.pi - 0.001:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.1

                    elif temp >= 0 and temp <= math.pi - 0.0001:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.01

                    elif temp < 0 and temp >= -math.pi + 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = 2

                    elif temp < 0 and temp <= -math.pi + 0.001:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.1


                    elif temp < 0 and temp <= -math.pi + 0.0001:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.01

                    else:
                        self.move.linear.x = 0
                        self.move.angular.z = 0
                        run = False

                    self.pubMove.publish(self.move)

                self.turret = Float32(0)
                self.pubTurret.publish(self.turret)

                self.zone.SetIsInZoneFlage(False)

            elif self.zone.GetZoneColor() == ZoneColor.RED:
                if self.zone.GetZoneNumber() == 1:
                    escapePositionX = -0.55
                    escapePositionY = -0.525

                    [escapeAngle, escapeRange] = self.CalculateEscapeAngleAndRangeColorZone1(escapePositionX, escapePositionY)

                elif self.zone.GetZoneNumber() == 2:
                    escapePositionX = -1.075
                    escapePositionY = -0.3

                    [escapeAngle, escapeRange] = self.CalculateEscapeAngleAndRange(escapePositionX, escapePositionY)


                #print("escape angle: ", escapeAngle)
                #print("escepe range: ", escapeRange)
                run = True
                while run and not self.robotStop:
                    if self.zone.GetZoneNumber() == 1:
                        [escapeAngle, escapeRange] = self.CalculateEscapeAngleAndRangeColorZone1(escapePositionX, escapePositionY)
                    else:
                        [escapeAngle, escapeRange] = self.CalculateEscapeAngleAndRange(escapePositionX, escapePositionY)

                    if escapeRange <= 0.0001:
                        self.move.linear.x = 0
                        self.move.angular.z = 0
                        run = False

                    elif escapeAngle >= 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.4

                    elif escapeAngle <= -0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.4

                    elif escapeAngle >= 0.001:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.01

                    elif escapeAngle <= -0.001:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.01

                    elif escapeRange >= 0.05:
                        self.move.linear.x = -0.2
                        self.move.angular.z = 0

                    elif escapeRange >= 0.001:
                        self.move.linear.x = -0.01
                        self.move.angular.z = 0

                    elif escapeRange > 0.00001:
                        self.move.linear.x = -0.0001
                        self.move.angular.z = 0

                    self.pubMove.publish(self.move)

                run = True
                while run and not self.robotStop:
                    self.GetPositionAndOrientationInZone()
                    temp = self.zone.GetOrientationRad()

                    if temp >= 0 and temp <= math.pi - 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.5

                    elif temp >= 0 and temp <= math.pi - 0.001:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.01

                    elif temp >= 0 and temp <= math.pi - 0.0001:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.001

                    elif temp < 0 and temp >= -math.pi + 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.5

                    elif temp < 0 and temp <= -math.pi + 0.001:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.01


                    elif temp < 0 and temp <= -math.pi + 0.0001:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.001

                    else:
                        self.move.linear.x = 0
                        self.move.angular.z = 0
                        run = False

                    self.pubMove.publish(self.move)

                self.turret = Float32(0)
                self.pubTurret.publish(self.turret)

                self.zone.SetIsInZoneFlage(False)

            return self.GetOrientationFromTag()

        return False

    def CalculateEscapeAngleAndRange(self, escapePositionX, escapePositionY):
        self.GetPositionAndOrientationInZone()
        
        dX = self.zone.GetPositionInZoneX() - escapePositionX
        dY = self.zone.GetPositionInZoneY() - escapePositionY
        
        range = math.sqrt(dX*dX + dY*dY)

        alpha = math.asin(dY/range)

        escapeAngle = self.zone.GetOrientationRad() + alpha
        #print("dX: {}, dY: {}, alpha: {}, orientation: {}".format(dX, dY, alpha, self.zone.GetOrientationRad()))
        return [escapeAngle, range]

    def CalculateEscapeAngleAndRangeColorZone1(self, escapePositionX, escapePositionY):
        self.GetPositionAndOrientationInZone()

        dX = self.zone.GetPositionInZoneX() - escapePositionX
        dY = self.zone.GetPositionInZoneY() - escapePositionY

        range = math.sqrt(dX * dX + dY * dY)

        alpha = math.acos(abs(dY) / range)

        if dX < 0:
            escapeAngle = self.zone.GetOrientationRad() - alpha
        else:
            escapeAngle = self.zone.GetOrientationRad() + alpha
        #print("dX: {}, dY: {}, alpha: {}, orientation: {}".format(dX, dY, alpha, self.zone.GetOrientationRad()))
        return [escapeAngle, range]


    def PutDownPuckOnPosition(self, positionX, positionY):
        if self.zone.GetZoneNumber() == 1:
            [angle, range] = self.CalculateAngleAndRangeToPositionColorZone1(positionX, positionY)
        else:
            [angle, range] = self.CalculateAngleAndRangeToPositionColorZone(positionX, positionY)


        if not self.CheckPositionIsEmpty(angle, range):
            return False

        #Dodac przesunicie pozycji bo robot ma swoja grubosc 
        run = True
        while run and not self.robotStop:
            if self.zone.GetZoneNumber() == 1:
                [angle, range] = self.CalculateAngleAndRangeToPositionColorZone1(positionX, positionY)
            else:
                [angle, range] = self.CalculateAngleAndRangeToPositionColorZone(positionX, positionY)

            #print("Angle: ", angle)
            #print("Range: ", range)
                 
            if range <= 0.2:
                self.move.linear.x = 0
                self.move.angular.z = 0
                run = False
                            
            elif angle >= 0.05:
                self.move.linear.x = 0
                self.move.angular.z = 0.4

            elif angle <= -0.05:
                self.move.linear.x = 0
                self.move.angular.z = -0.4
                
            elif angle >= 0.001:
                self.move.linear.x = 0
                self.move.angular.z = 0.01
                            
            elif angle <= -0.001:
                self.move.linear.x = 0
                self.move.angular.z = -0.01

            elif range >= 0.22:
                self.move.linear.x = 0.2
                self.move.angular.z = 0

            elif range >= 0.01:
                self.move.linear.x = 0.01
                self.move.angular.z = 0

                
            self.pubMove.publish(self.move)

        print("Gripper")
        self.gripper = False
        self.pubGripper.publish(self.gripper)

        if self.robotStop:
            return False
        
        return True
        

    def CalculateAngleAndRangeToPositionColorZone(self, positionX, positionY):
        self.GetPositionAndOrientationInZone()
        
        dX = self.zone.GetPositionInZoneX() - positionX
        dY = self.zone.GetPositionInZoneY() - positionY


        range = math.sqrt(dX*dX + dY*dY)

        alpha = math.asin(dY/range)

        angle = self.zone.GetOrientationRad() - alpha
        #print("dX: {}, dY: {}, alpha: {}, orientation: {}".format(dX,dY,alpha, self.zone.GetOrientationRad()))
        return [angle, range]

    def CalculateAngleAndRangeToPositionColorZone1(self, positionX, positionY):
        self.GetPositionAndOrientationInZone()

        dX = self.zone.GetPositionInZoneX() - positionX
        dY = self.zone.GetPositionInZoneY() - positionY

        range = math.sqrt(dX * dX + dY * dY)

        alpha = math.acos(abs(dY) / range)

        if dX > 0:
            angle = self.zone.GetOrientationRad() - alpha
        else:
            angle = self.zone.GetOrientationRad() + alpha

        #print("dX: {}, dY: {}, alpha: {}, orientation: {}".format(dX, dY, alpha, self.zone.GetOrientationRad()))
        return [angle, range]

    def CheckPositionIsEmpty(self, angle, range):
        puckListClone = self.puckList

        for puck in puckListClone.pucks:
            if (puck.range - range) <= 0.01 and (puck.angle - angle) <= 0.001:
                return False

        return True

    
def main():
    rospy.init_node('testBot', anonymous=True)
    if len(sys.argv) > 1:
           botName = sys.argv[1]
    else:
           botName = "bot"

    print(botName)
    TransportRobot(botName)
    rospy.spin()
    

if __name__ == '__main__':
    main()
