// ROS Stuff #include "ros/ros.h"
#include "argos_bridge/Puck.h"
#include "argos_bridge/PuckList.h"
#include "argos_bridge/Proximity.h"
#include "argos_bridge/ProximityList.h"
#include "argos_bridge/DistScan.h"
#include "argos_bridge/DistScanList.h"

/* Include the controller definition */
#include "argos_ros_bot_lit.h"
/* Function definitions for XML parsing */
#include <argos3/core/utility/configuration/argos_configuration.h>
/* 2D vector definition */
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/datatypes/color.h>

#include <iostream>
#include <sstream>

#include <ros/callback_queue.h>

using namespace std;
using namespace argos_bridge;

// Initialize ROS node.  There will be only one ROS node no matter how many robots are created in
// ARGoS.  However, we will have one instance of the CArgosRosBot class for each ARGoS robot.
ros::NodeHandle* initROS() {
  int argc = 0;
  char *argv = (char *) "";
  ros::init(argc, &argv, "argos_bridge");
  return new ros::NodeHandle();
}

ros::NodeHandle* CArgosRosBotLit::nodeHandle = initROS();

/****************************************/
/****************************************/

CArgosRosBotLit::CArgosRosBotLit() :
  m_pcWheels(NULL),
  m_pcProximity(NULL),
  m_pcOmniCam(NULL),
  m_pcGripper(NULL),
  m_pcLeds(NULL),
  //m_pcDistScan(NULL),
  //m_pcDistScanAcr(NULL),
  //m_pcMotoGround(NULL),
  //m_pcBaseGround(NULL),
  m_pcPosition(NULL),
  m_pcTurretAcr(NULL),
  m_pcTurretEncoder(NULL),

  stopWithoutSubscriberCount(10),
  stepsSinceCallback(0),
  leftSpeed(0),
  rightSpeed(0),
  distScanRun(true),
  gripping(false)
{
}

void CArgosRosBotLit::Init(TConfigurationNode& t_node) {
  // Create the topics to publish
  stringstream puckListTopic, proximityTopic, distScanTopic, motoGroundTopic, baseGroundTopic, positionTopic, turretEncoderTopic;
  puckListTopic << "/" << GetId() << "/puck_list";
  proximityTopic << "/" << GetId() << "/proximity";
  //distScanTopic << "/" << GetId() << "/distScan";
  //motoGroundTopic << "/" << GetId() << "/motoGround";
  //baseGroundTopic << "/" << GetId() << "/baseGround";
  positionTopic << "/" << GetId() << "/position";
  turretEncoderTopic << "/" << GetId() << "/turretEncoder";
  puckListPub = nodeHandle->advertise<PuckList>(puckListTopic.str(), 1);
  proximityPub = nodeHandle->advertise<ProximityList>(proximityTopic.str(), 1);
  //distScanPub = nodeHandle->advertise<DistScanList>(distScanTopic.str(), 1);
  //motoGroundPub = nodeHandle->advertise<MotoGroundList>(motoGroundTopic.str(), 1);
  //baseGroundPub = nodeHandle->advertise<BaseGroundList>(baseGroundTopic.str(), 1);
  positionPub = nodeHandle->advertise<Position>(positionTopic.str(), 1);
  turretEncoderPub = nodeHandle->advertise<std_msgs::Float32>(turretEncoderTopic.str(), 1);

  // Create the subscribers
  stringstream cmdVelTopic, ledsTopic, distScanAcrTopic, gripperTopic, turretAcrTopic;
  cmdVelTopic << "/" << GetId() << "/cmd_vel";
  gripperTopic << "/" << GetId() << "/gripper";
  ledsTopic << "/" << GetId() << "/leds";
  //distScanAcrTopic << "/" << GetId() << "/distScanController";
  turretAcrTopic << "/" << GetId() << "/turretController";

  cmdVelSub = nodeHandle->subscribe(cmdVelTopic.str(), 1, &CArgosRosBotLit::cmdVelCallback, this);
  gripperSub = nodeHandle->subscribe(gripperTopic.str(), 1, &CArgosRosBotLit::gripperCallback, this);
  ledsSub = nodeHandle->subscribe(ledsTopic.str(), 1, &CArgosRosBotLit::ledsCallback, this);
  //distScanAcrSub = nodeHandle->subscribe(distScanAcrTopic.str(), 1, &CArgosRosBot::distScanAcrCallback, this);
  turretAcrSub = nodeHandle->subscribe(turretAcrTopic.str(), 1, &CArgosRosBotLit::turretAcrCallback, this);
 
  // Get sensor/actuator handles
  m_pcWheels = GetActuator<CCI_DifferentialSteeringActuator>("differential_steering");
  m_pcProximity = GetSensor<CCI_FootBotProximitySensor>("footbot_proximity");
  m_pcOmniCam = GetSensor<CCI_ColoredBlobOmnidirectionalCameraSensor>("colored_blob_omnidirectional_camera");
  m_pcGripper = GetActuator<CCI_FootBotGripperActuator>("footbot_gripper");
  m_pcLeds = GetActuator<CCI_LEDsActuator>("leds");
  //m_pcDistScan = GetSensor<CCI_FootBotDistanceScannerSensor>("footbot_distance_scanner");
  //m_pcDistScanAcr = GetActuator<CCI_FootBotDistanceScannerActuator>("footbot_distance_scanner");
  //m_pcMotoGround = GetSensor<CCI_FootBotMotorGroundSensor>("footbot_motor_ground");
  //m_pcBaseGround = GetSensor<CCI_FootBotBaseGroundSensor>("footbot_base_ground");
  m_pcPosition = GetSensor<CCI_PositioningSensor>("positioning");
  m_pcTurretAcr = GetActuator<CCI_FootBotTurretActuator>("footbot_turret");
  m_pcTurretEncoder = GetSensor<CCI_FootBotTurretEncoderSensor>("footbot_turret_encoder");

  m_pcOmniCam->Enable();
  //m_pcDistScanAcr->SetAngle(CRadians::PI_OVER_TWO);
  //m_pcDistScanAcr->Enable();
  //m_pcDistScanAcr->SetRPM(30);
  //m_pcTurretAcr->SetMode(CCI_FootBotTurretActuator::ETurretModes::MODE_OFF);
  m_pcTurretAcr->SetPositionControlMode();
  m_pcTurretAcr->SetActiveWithRotation(CRadians(0));

  /*
   * Parse the configuration file
   *
   * The user defines this part. Here, the algorithm accepts three
   * parameters and it's nice to put them in the config file so we don't
   * have to recompile if we want to try other settings.
   */
  GetNodeAttributeOrDefault(t_node, "stopWithoutSubscriberCount", stopWithoutSubscriberCount, stopWithoutSubscriberCount);
}

// Compares pucks for sorting purposes.  We sort by angle.
bool puckComparator(Puck a, Puck b) {
  return a.angle < b.angle;
}

void CArgosRosBotLit::ControlStep() {
  const CCI_ColoredBlobOmnidirectionalCameraSensor::SReadings& camReads = m_pcOmniCam->GetReadings();
  PuckList puckList;
  puckList.n = camReads.BlobList.size();
  for (size_t i = 0; i < puckList.n; ++i) {
    Puck puck;
    puck.color.r = camReads.BlobList[i]->Color.GetRed();
    puck.color.g = camReads.BlobList[i]->Color.GetGreen();
    puck.color.b = camReads.BlobList[i]->Color.GetBlue();
    puck.color.a = camReads.BlobList[i]->Color.GetAlpha();
    puck.range = camReads.BlobList[i]->Distance;
    // Make the angle of the puck in the range [-PI, PI].  This is useful for
    // tasks such as homing in on a puck using a simple controller based on
    // the sign of this angle.
    puck.angle = camReads.BlobList[i]->Angle.SignedNormalize().GetValue();
    puckList.pucks.push_back(puck);
  }

  // Sort the puck list by angle.  This is useful for the purposes of extracting meaning from
  // the local puck configuration (e.g. fitting a lines to the detected pucks).
  sort(puckList.pucks.begin(), puckList.pucks.end(), puckComparator);

  puckListPub.publish(puckList);

  /* Get readings from proximity sensor */
  const CCI_FootBotProximitySensor::TReadings& tProxReads = m_pcProximity->GetReadings();
  ProximityList proxList;
  proxList.n = tProxReads.size();
  for (size_t i = 0; i < proxList.n; ++i) {
    Proximity prox;
    prox.value = tProxReads[i].Value;
    prox.angle = tProxReads[i].Angle.GetValue();
    proxList.proximities.push_back(prox);

//cout << GetId() << ": value: " << prox.value << ": angle: " << prox.angle << endl;
  }

  proximityPub.publish(proxList);

  /* Get readings from distance scanner sensor */
  /*
  if(distScanRun == true)
  {
    const CCI_FootBotDistanceScannerSensor::TReadingsMap tDistReads = m_pcDistScan->GetReadingsMap();
    DistScanList scanList;
    scanList.n = tDistReads.size();
    CCI_FootBotDistanceScannerSensor::TReadingsMap::const_iterator it = tDistReads.begin();
    for(; it != tDistReads.end(); it++) {
      DistScan scan;
      scan.range = it->second;
      scan.angle = it->first.GetValue();
      scanList.scan.push_back(scan);
    }

    distScanPub.publish(scanList);
  }
  */

  /*
  const CCI_FootBotMotorGroundSensor::TReadings tMotoGroundReads = m_pcMotoGround->GetReadings();
  MotoGroundList motoList;
  motoList.n = tMotoGroundReads.size();
  for(size_t i = 0; i < motoList.n; i++)
  {
    MotoGround motoG;
    motoG.value = tMotoGroundReads[i].Value;
    motoG.offset_x = tMotoGroundReads[i].Offset.GetX();
    motoG.offset_y = tMotoGroundReads[i].Offset.GetY();
    motoList.motoGrounds.push_back(motoG);
  }

  motoGroundPub.publish(motoList);
  */
  /*
  const CCI_FootBotBaseGroundSensor::TReadings tBaseGroundReads = m_pcBaseGround->GetReadings();
  BaseGroundList baseList;
  baseList.n = tBaseGroundReads.size();
  for(size_t i = 0; i < baseList.n; i++)
  {
    BaseGround baseG;
    baseG.value = tBaseGroundReads[i].Value;
    baseG.offset_x = tBaseGroundReads[i].Offset.GetX();
    baseG.offset_y = tBaseGroundReads[i].Offset.GetY();
    baseList.baseGrounds.push_back(baseG);
  }

  baseGroundPub.publish(baseList);
  */
  
  const CCI_PositioningSensor::SReading tPositionReads = m_pcPosition->GetReading();
  Position position;
  position.position.x = tPositionReads.Position.GetX();
  position.position.y = tPositionReads.Position.GetY();
  position.position.z = tPositionReads.Position.GetZ();
  CRadians x,y,z;
  tPositionReads.Orientation.ToEulerAngles(z, y, x);
  position.orientation.z = z.GetValue();
  position.orientation.y = y.GetValue();
  position.orientation.x = x.GetValue();

  positionPub.publish(position);

  const CRadians tRotationReads = m_pcTurretEncoder->GetRotation();
  std_msgs::Float32 rotationTurret;
  rotationTurret.data = tRotationReads.GetValue();

  turretEncoderPub.publish(rotationTurret);

  // Wait for any callbacks to be called.
  ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));

  // If we haven't heard from the subscriber in a while, set the speed to zero.
  if (stepsSinceCallback > stopWithoutSubscriberCount) {
    leftSpeed = 0;
    rightSpeed = 0;
  } else {
    stepsSinceCallback++;
  }

  m_pcWheels->SetLinearVelocity(leftSpeed, rightSpeed);
}

void CArgosRosBotLit::cmdVelCallback(const geometry_msgs::Twist& twist) {
  cout << "cmdVelCallback: " << GetId() << endl;

  Real v = twist.linear.x;  // Forward speed
  Real w = twist.angular.z; // Rotational speed

  // Use the kinematics of a differential-drive robot to derive the left
  // and right wheel speeds.
  leftSpeed = (v - HALF_BASELINE * w) / WHEEL_RADIUS;
  rightSpeed = (v + HALF_BASELINE * w) / WHEEL_RADIUS;

  stepsSinceCallback = 0;
}


void CArgosRosBotLit::gripperCallback(const std_msgs::Bool& value) {
  cout << "gripperCallback: " << GetId() << endl;

  if (gripping && !value.data) {
    // Release the gripper
    m_pcGripper->Unlock();
    cout << "Gripper unlock" << endl;
    gripping = false;
  }

  if (!gripping && value.data) {
    // Activate gripper
    m_pcGripper->LockPositive();
    cout << "Gripper lock" << endl;
    gripping = true;
  }

  stepsSinceCallback = 0;
}

void CArgosRosBotLit::ledsCallback(const LedsColor color)
{
   cout << "ledsCallback: " << GetId() <<endl;

   float f_r = color.color.r;
   if(f_r > 1) f_r = 1;
   else if(f_r < 0) f_r = 0;

   float f_g = color.color.g;
   if(f_g > 1) f_g = 1;
   else if(f_g < 0) f_g = 0;

   float f_b = color.color.b;
   if(f_b > 1) f_b = 1;
   else if(f_b < 0) f_b = 0;

   float f_a = color.color.a;
   if(f_a > 1) f_a = 1;
   else if(f_a < 0) f_a = 0;

   CColor c_color = CColor((UInt8)(f_r*255),(UInt8)(f_g*255),(UInt8)(f_b*255),(UInt8)(f_a*255));
   m_pcLeds->SetSingleColor(color.led_number, c_color);
   m_pcLeds->SetSingleIntensity(color.led_number, color.intensity);

}

/*
void CArgosRosBot::distScanAcrCallback(const std_msgs::Bool& value)
{
   cout << "distScanArcCallback: " << GetId() <<endl;

   if(value.data == true && distScanRun == false)
   {
     m_pcDistScanAcr->Enable();
     distScanRun = true;

   }else if(value.data == false && distScanRun == true)
   {
     m_pcDistScanAcr->Disable();
     distScanRun = false;
   }
}
*/

void CArgosRosBotLit::turretAcrCallback(const std_msgs::Float32& rotation)
{
  cout << "turretAcrCallback: " << GetId() << " value: " << CRadians((rotation.data)) << endl;
  m_pcTurretAcr->SetRotation(CRadians((rotation.data)));
  cout << "Set rotation" << endl;
}


/*
* This statement notifies ARGoS of the existence of the controller.
* It binds the class passed as first argument to the string passed as
* second argument.
* The string is then usable in the configuration file to refer to this
* controller.
 * When ARGoS reads that string in the configuration file, it knows which
 * controller class to instantiate.
 * See also the configuration files for an example of how this is used.
 */
REGISTER_CONTROLLER(CArgosRosBotLit, "argos_ros_bot_controller_lit")
