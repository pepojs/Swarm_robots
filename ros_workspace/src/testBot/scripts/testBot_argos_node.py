#!/usr/bin/python3

import sys
import enum
import math

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

class OrientationFromTag(enum.IntEnum):
    NOT_SPECIFIED = -1
    NORTH = 0
    EAST = 1
    SOUTH = 2
    WEST = 3
    
class ZoneColor(enum.Enum):
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
        self.move = Twist()
        self.gripper = Bool(False)

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
        
        rospy.Subscriber("/" + botName + "/proximity", ProximityList, self.callbackProximity)
        rospy.Subscriber("/" + botName + "/baseGround", BaseGroundList, self.callbackBaseGround)
        rospy.Subscriber("/" + botName + "/motoGround", MotoGroundList, self.callbackMotoGround)
        rospy.Subscriber("/" + botName + "/puck_list", PuckList, self.callbackPuckList)
        rospy.Subscriber("/" + botName + "/position", Position, self.callbackPosition)
        rospy.Subscriber("/" + botName + "/robotStop", Bool, self.callbackRobotStop)
        
        while self.newMotoGroundMsg == False:
            pass

        #self.TakeNearestPuck()
        #self.EnterToZone(ZoneColor.BLACK)
        #self.zone.SetZoneNumber(1)
        #self.zone.SetIsInZoneFlage(True)
        #self.GetPositionAndOrientationInZone()
        
        self.EnterToZone(ZoneColor.BLACK)
        self.TakeNearestPuck()
        self.EscapeFromZone()
          
        self.MoveToForward()
        self.MoveToForward()
        self.MoveToForward()
        self.RotateRight()
        self.MoveToForward()
        self.MoveToForward()
        self.MoveToForward()
        self.MoveToForward()
        
        self.EnterToZone(ZoneColor.RED)
        #self.PutDownPuckOnPosition(-2.2, 2.2)
                

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

    def GetPuckColor(self):
        return self.puckColor
        
    def GetOrientationFromTag(self):
        if (self.motoGround[0].value > 0.49 and self.motoGround[0].value < 0.51) and \
           (self.motoGround[3].value > 0.44 and self.motoGround[3].value < 0.46):
            return OrientationFromTag.EAST

        elif (self.motoGround[0].value > 0.44 and self.motoGround[0].value < 0.46) and \
             (self.motoGround[3].value > 0.34 and self.motoGround[3].value < 0.36):
            return OrientationFromTag.SOUTH

        elif (self.motoGround[0].value > 0.34 and self.motoGround[0].value < 0.36) and \
             (self.motoGround[3].value > 0.39 and self.motoGround[3].value < 0.41):
            return OrientationFromTag.WEST

        elif (self.motoGround[0].value > 0.39 and self.motoGround[0].value < 0.41) and \
             (self.motoGround[3].value > 0.49 and self.motoGround[3].value < 0.51):
            return OrientationFromTag.NORTH

        else:
            return OrientationFromTag.NOT_SPECIFIED
        
        
        
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
                    print('Robot: {} got lost'.format(self.robotName))
                    
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


    def RotateLeft(self):
        if abs(self.motoGround[0].value - self.motoGround[1].value) > 0.01 and \
           abs(self.motoGround[1].value - self.motoGround[2].value) > 0.01 and \
           abs(self.motoGround[2].value - self.motoGround[3].value) > 0.01 and \
           abs(self.motoGround[3].value - self.motoGround[0].value) > 0.01 :

            orientationTag = self.GetOrientationFromTag()

            if orientationTag == OrientationFromTag.NORTH:
                finishOrientationTag = OrientationFromTag.WEST
            else:
                finishOrientationTag = orientationTag - 1

            if finishOrientationTag == OrientationFromTag.NORTH:
                finishOrientation = 0;
            elif finishOrientationTag == OrientationFromTag.EAST:
                finishOrientation = -math.pi/2;

            elif finishOrientationTag == OrientationFromTag.SOUTH:
                finishOrientation = math.pi;

            elif finishOrientationTag == OrientationFromTag.WEST:
                finishOrientation = math.pi/2;

                      
        run = True
        while run and not self.robotStop:
            if self.newMotoGroundMsg == True:
                self.newMotoGroundMsg = False
            
                if finishOrientationTag != self.GetOrientationFromTag():
                    self.move.linear.x = 0
                    self.move.angular.z = 0.8
                
                elif abs(self.motoGround[0].value - self.motoGround[1].value) <= 0.03 or \
                     abs(self.motoGround[1].value - self.motoGround[2].value) <= 0.03 or \
                     abs(self.motoGround[2].value - self.motoGround[3].value) <= 0.03 or \
                     abs(self.motoGround[3].value - self.motoGround[0].value) <= 0.03:
                    self.move.linear.x = 0
                    self.move.angular.z = 0.6

                elif finishOrientationTag == self.GetOrientationFromTag():
                   
                    
                    if finishOrientationTag != OrientationFromTag.SOUTH and \
                       finishOrientation - self.position.orientation.z > 0.0116 and \
                       finishOrientation - self.position.orientation.z <= 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.05

                    elif finishOrientationTag == OrientationFromTag.SOUTH and \
                         finishOrientation - self.position.orientation.z > -2*math.pi + 0.0116 and \
                         finishOrientation - self.position.orientation.z <= -2*math.pi + 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.05

                    elif finishOrientation - self.position.orientation.z < -0.0116 and\
                         finishOrientation - self.position.orientation.z >= -0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.05

                    elif (self.baseGround[1].value >= 0.99 and self.baseGround[3].value >= 0.99 and \
                          self.baseGround[5].value >= 0.99 and self.baseGround[7].value >= 0.99) or\
                          finishOrientation - self.position.orientation.z < -0.012 or \
                         (finishOrientationTag != OrientationFromTag.SOUTH and \
                          finishOrientation - self.position.orientation.z > 0.012) or \
                         (finishOrientationTag == OrientationFromTag.SOUTH and \
                          finishOrientation - self.position.orientation.z > -2*math.pi + 0.012):
                        self.move.linear.x = 0
                        self.move.angular.z = 0.0
                        self.pubMove.publish(self.move)
                        run = False
                        break
                        
                    else:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.15

            
                self.pubMove.publish(self.move)

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
                finishOrientation = 0;
            elif finishOrientationTag == OrientationFromTag.EAST:
                finishOrientation = -math.pi/2;

            elif finishOrientationTag == OrientationFromTag.SOUTH:
                finishOrientation = math.pi;

            elif finishOrientationTag == OrientationFromTag.WEST:
                finishOrientation = math.pi/2;
            
        run = True
        while run and not self.robotStop:
            if self.newMotoGroundMsg == True:
                self.newMotoGroundMsg = False

                if finishOrientationTag != self.GetOrientationFromTag():
                    self.move.linear.x = 0
                    self.move.angular.z = -0.8
                
                elif abs(self.motoGround[0].value - self.motoGround[1].value) <= 0.03 or \
                     abs(self.motoGround[1].value - self.motoGround[2].value) <= 0.03 or \
                     abs(self.motoGround[2].value - self.motoGround[3].value) <= 0.03 or \
                     abs(self.motoGround[3].value - self.motoGround[0].value) <= 0.03:
                    self.move.linear.x = 0
                    self.move.angular.z = -0.6

                elif finishOrientationTag == self.GetOrientationFromTag():
                   
                    
                    if finishOrientationTag != OrientationFromTag.SOUTH and \
                       finishOrientation - self.position.orientation.z > 0.0116 and \
                       finishOrientation - self.position.orientation.z <= 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.05

                    elif finishOrientationTag == OrientationFromTag.SOUTH and \
                         finishOrientation - self.position.orientation.z > -2*math.pi + 0.0116 and \
                         finishOrientation - self.position.orientation.z <= -2*math.pi + 0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = 0.05

                    elif finishOrientation - self.position.orientation.z < -0.0116 and\
                         finishOrientation - self.position.orientation.z >= -0.05:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.05

                    elif self.baseGround[1].value >= 0.99 and self.baseGround[3].value >= 0.99 and \
                         self.baseGround[5].value >= 0.99 and self.baseGround[7].value >= 0.99 or\
                          finishOrientation - self.position.orientation.z < -0.012 or \
                         (finishOrientationTag != OrientationFromTag.SOUTH and \
                          finishOrientation - self.position.orientation.z > 0.012) or \
                         (finishOrientationTag == OrientationFromTag.SOUTH and \
                          finishOrientation - self.position.orientation.z > -2*math.pi + 0.012):
                        self.move.linear.x = 0
                        self.move.angular.z = 0.0
                        self.pubMove.publish(self.move)
                        run = False
                        break
                        
                    else:
                        self.move.linear.x = 0
                        self.move.angular.z = -0.15

            
                self.pubMove.publish(self.move)
    

    def TakeNearestPuck(self):            
        [puckPositionX, puckPositionY] = self.CalculateNearestPuckPosition()

        [puckAngle, puckRange] = self.CalculatePuckAngleAndRange(puckPositionX, puckPositionY)
        print("puck angle: ", puckAngle)
        print("puck range: ", puckRange)
        run = True
        while run and not self.robotStop:
            [puckAngle, puckRange] = self.CalculatePuckAngleAndRange(puckPositionX, puckPositionY)

                 
            if puckRange <= 0.15:
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

        
    def CalculateNearestPuckPosition(self):
        self.GetPositionAndOrientationInZone()
        puckListClone = self.puckList
        nearestPuck = -1
        minRange = 1000
        i = -1
        
        for puck in puckListClone.pucks:
            i = i + 1
                
            if (puck.color.r == 165 and puck.color.g == 42 and puck.color.b == 42) or \
               (puck.color.r == 255 and puck.color.g == 140 and puck.color.b == 0):
                continue
            else:
                if puck.range <= minRange:
                    nearestPuck = i
                    minRange = puck.range


        self.SetPuckColor(puckListClone.pucks[nearestPuck].color)
         
        puckAngle = puckListClone.pucks[nearestPuck].angle
        puckRange = puckListClone.pucks[nearestPuck].range*0.01

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
        print("angle: ", puckAngle)
        return [puckAngle, range]

    
        
    def EnterToZone(self, zoneColor: ZoneColor):
        self.zone.SetZoneColor(zoneColor)
        self.zone.SetIsInZoneFlage(True)
        print(zoneColor)
        if zoneColor == ZoneColor.BLACK:
            
            if (self.motoGround[0].value >= 0.39 and self.motoGround[0].value <= 0.41) and \
               (self.motoGround[3].value >= 0.49 and self.motoGround[3].value <= 0.51):
                
                self.zone.SetZoneNumber(1)
                self.GetPositionAndOrientationInZone()

        if zoneColor == ZoneColor.RED:
            print("Red")
            if(self.motoGround[0].value >= 0.34 and self.motoGround[0].value <= 0.36) and \
              (self.motoGround[3].value >= 0.39 and self.motoGround[3].value <= 0.41):
                print("Red1")
                self.zone.SetZoneNumber(1)
                self.GetPositionAndOrientationInZone()
                print("enter to red zone")
                
    def GetPositionAndOrientationInZone(self):

        if self.zone.IsInZone():

            if self.zone.GetZoneColor() == ZoneColor.BLACK:
                if self.zone.GetZoneNumber() == 1:

                    
                    [centerLanternRange, centerLanternAngle, secondLanternRange, secondLanternAngle] \
                        = self.GetOrangeBrownLantern()

                    if centerLanternRange == -1 or secondLanternRange == -1:
                        print("Can't get position in zone")
                        return
                    if centerLanternRange == -2 and secondLanternRange == -2:
                        #print("Not new position")
                        return

                    centerSecondRange = 0.525
                                       
                    alpha = math.acos((centerSecondRange*centerSecondRange \
                                       + centerLanternRange*centerLanternRange \
                                       - secondLanternRange*secondLanternRange) \
                                      /(2 *centerSecondRange*centerLanternRange))

                    self.zone.SetPositionInZoneY(-math.sin(alpha)*centerLanternRange)
                    self.zone.SetPositionInZoneX(-math.cos(alpha)*centerLanternRange)
                    print("position in zone x: ", self.zone.GetPositionInZoneX())
                    print("position in zone y: ", self.zone.GetPositionInZoneY())
            
                    beta = math.asin(abs(self.zone.GetPositionInZoneY())/centerLanternRange)
                    self.zone.SetOrientationRad(centerLanternAngle - beta)
                    print("orientation to axis x: ", self.zone.GetOrientationRad())

            if self.zone.GetZoneColor() == ZoneColor.RED:
               if self.zone.GetZoneNumber() == 1:

                   [centerLanternRange, centerLanternAngle, secondLanternRange, secondLanternAngle] \
                        = self.GetOrangeBrownLantern()

                   if centerLanternRange == -1 or secondLanternRange == -1:
                       print("Can't get position in zone")
                       return
                   if centerLanternRange == -2 and secondLanternRange == -2:
                       #print("Not new position")
                       return

                   #centerSecondRange = 1.16297033496
                   centerSecondRange = 0.525
                   print(((centerSecondRange*centerSecondRange \
                                      + centerLanternRange*centerLanternRange \
                                      - secondLanternRange*secondLanternRange) \
                                     /(2 *centerSecondRange*centerLanternRange)))
                   alpha = math.acos((centerSecondRange*centerSecondRange \
                                      + centerLanternRange*centerLanternRange \
                                      - secondLanternRange*secondLanternRange) \
                                     /(2 *centerSecondRange*centerLanternRange))

                   self.zone.SetPositionInZoneY(2.4-math.sin(alpha)*centerLanternRange)
                   self.zone.SetPositionInZoneX(-2.95+math.cos(alpha)*centerLanternRange)
                   print("position in zone x: ", self.zone.GetPositionInZoneX())
                   print("position in zone y: ", self.zone.GetPositionInZoneY())
            
                   beta = math.asin(abs(2.4-self.zone.GetPositionInZoneY())/centerLanternRange)
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

    
    def EscapeFromZone(self):
        if self.zone.IsInZone():
        
            self.GetPositionAndOrientationInZone()
            if self.zone.GetZoneColor() == ZoneColor.BLACK:
                if self.zone.GetZoneNumber() == 1:
                    escapePositionX = -0.525
                    escapePositionY = -0.3
                    
                    [escapeAngle, escapeRange] = self.CalculateEscapeAngleAndRange(escapePositionX, escapePositionY)
                    print("escape angle: ", escapeAngle)
                    print("escepe range: ", escapeRange)
                    run = True
                    while run and not self.robotStop:
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

                    self.zone.SetIsInZoneFlage(False)

                        
    def CalculateEscapeAngleAndRange(self, escapePositionX, escapePositionY):
        self.GetPositionAndOrientationInZone()
        
        dX = self.zone.GetPositionInZoneX() - escapePositionX
        dY = self.zone.GetPositionInZoneY() - escapePositionY
        
        range = math.sqrt(dX*dX + dY*dY)

        alpha = math.asin(dY/range)

        escapeAngle = self.zone.GetOrientationRad() + alpha
        
        return [escapeAngle, range]


    def PutDownPuckOnPosition(self, positionX, positionY):
        [angle, range] = self.CalculateAngleAndRangeToPosition(positionX, positionY)

        if not self.CheckPositionIsEmpty(angle, range):
            return False

        #Dodac przesunicie pozycji bo robot ma swoja grubosc 
        run = True
        while run and not self.robotStop:
            [angle, range] = self.CalculateAngleAndRangeToPosition(positionX, positionY)
            print("Angle: ", angle)
            print("Range: ", range)
                 
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

        self.gripper = False
        self.pubGripper.publish(self.gripper)

        if self.robotStop:
            return False
        
        return True
        

    def CalculateAngleAndRangeToPosition(self, positionX, positionY):
        self.GetPositionAndOrientationInZone()
        
        dX = self.zone.GetPositionInZoneX() - positionX
        dY = self.zone.GetPositionInZoneY() - positionY
        
        range = math.sqrt(dX*dX + dY*dY)

        alpha = math.asin(dY/range)

        angle = self.zone.GetOrientationRad() - alpha
        return [angle, range]

    def CheckPositionIsEmpty(self, angle, range):
        puckListClone = self.puckList

        for puck in puckListClone.pucks:
            if (puck.range - range) <= 0.01 and (puck.angle - angle) <= 0.001:
                return False

        return True

    
def main():
    rospy.init_node('testBot')
    if len(sys.argv) > 1:
           botName = sys.argv[1]
    else:
           botName = "bot"

    print(botName)
    TransportRobot(botName)
    rospy.spin()
    

if __name__ == '__main__':
    main()
