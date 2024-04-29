#include "ros/ros.h"
#include "std_msgs/String.h"
#include "virtuose/out_virtuose_status.h"
#include "virtuose/out_virtuose_pose.h"
#include "virtuose/out_virtuose_physical_pose.h"
#include "virtuose/out_virtuose_speed.h"
#include "virtuose/out_virtuose_force.h"
#include "virtuose/in_virtuose_pose.h"
#include "virtuose/in_virtuose_speed.h"
#include "virtuose/in_virtuose_force.h"
#include "virtuose/virtuose_calibrate.h"
#include "virtuose/virtuose_admittance.h"
#include "virtuose/virtuose_impedance.h"
#include "virtuose/virtuose_reset.h"

#include "VirtuoseAPI.h"

/* Current state of the virtuose node */
#define STATE_UNKNOWN 0
#define STATE_READY 1
#define STATE_ERROR 2
#define STATE_CALIB 3
#define STATE_CART_IMP 4
#define STATE_CART_ADM 5
uint8_t _state = STATE_UNKNOWN;

/* VirtContext */
VirtContext _VC = NULL;

/* Publishers */
ros::Publisher* _out_virtuose_status;
ros::Publisher* _out_virtuose_pose;
ros::Publisher* _out_virtuose_physical_pose;
ros::Publisher* _out_virtuose_speed;
ros::Publisher* _out_virtuose_force;
ros::Subscriber* _in_virtuose_pose;

/* Setpoints */
float set_pose[7] = { 0, 0, 0, 0, 0, 0, 1 };
uint64_t set_pose_date = 0;
float set_speed[6];
uint64_t set_speed_date = 0;
float set_force[6];
uint64_t set_force_date = 0;

/* Dates */
uint64_t start_loop_date = 0;

/* Storage for current client ID */
uint32_t _client_id = 0;

uint32_t generate_client_id()
{
  srand(ros::Time::now().toNSec());
  _client_id = (uint32_t)rand();
  return _client_id;
}

// Callback for the topic in_virtuose_pose
void in_virtuose_poseCB(const virtuose::in_virtuose_pose::ConstPtr& msg)
{
  // Check client id
  if (msg->client_id != _client_id)
    return;
  // Store last set_pose date
  set_pose_date = msg->header.stamp.toNSec();
  // Store set_pose
  set_pose[0] = msg->virtuose_pose.translation.x;
  set_pose[1] = msg->virtuose_pose.translation.y;
  set_pose[2] = msg->virtuose_pose.translation.z;
  set_pose[3] = msg->virtuose_pose.rotation.x;
  set_pose[4] = msg->virtuose_pose.rotation.y;
  set_pose[5] = msg->virtuose_pose.rotation.z;
  set_pose[6] = msg->virtuose_pose.rotation.w;
  // Normalize quaternion
  float norm_quat = sqrt(set_pose[3]*set_pose[3]+set_pose[4]*set_pose[4]+set_pose[5]*set_pose[5]+set_pose[6]*set_pose[6]);
  if (norm_quat == 0.0)
  {
    set_pose[6] = 1.0;
  }
  else
  {
    set_pose[3] /= norm_quat;
    set_pose[4] /= norm_quat;
    set_pose[5] /= norm_quat;
    set_pose[6] /= norm_quat;
  }
}

// Callback for topic in_virtuose_speed
void in_virtuose_speedCB(const virtuose::in_virtuose_speed::ConstPtr& msg)
{
  // Check client id
  if (msg->client_id != _client_id)
    return;
  // Store last set_speed date
  set_speed_date = msg->header.stamp.toNSec();
  set_speed[0] = msg->virtuose_speed.linear.x;
  set_speed[1] = msg->virtuose_speed.linear.y;
  set_speed[2] = msg->virtuose_speed.linear.z;
  set_speed[3] = msg->virtuose_speed.angular.x;
  set_speed[4] = msg->virtuose_speed.angular.y;
  set_speed[5] = msg->virtuose_speed.angular.z;
}

// Callback for topic in_virtuose_force
void in_virtuose_forceCB(const virtuose::in_virtuose_force::ConstPtr& msg)
{
  // Check client id
  if (msg->client_id != _client_id)
    return;
  // Store last set_force date
  set_force_date = msg->header.stamp.toNSec();
  set_force[0] = msg->virtuose_force.force.x;
  set_force[1] = msg->virtuose_force.force.y;
  set_force[2] = msg->virtuose_force.force.z;
  set_force[3] = msg->virtuose_force.torque.x;
  set_force[4] = msg->virtuose_force.torque.y;
  set_force[5] = msg->virtuose_force.torque.z;
}

// Callback for service virtuose_calibrate
bool calibrateCB(virtuose::virtuose_calibrate::Request& request, virtuose::virtuose_calibrate::Response &response)
{
  printf("Performing calibration on ip address %s\n", request.ip_address.c_str());
  ros::Duration(1.0).sleep();
  // Change state to CALIB
  virtuose::out_virtuose_status status;
  status.header.stamp = ros::Time::now();
  status.header.frame_id = "/Calibration";
  status.state = _state = STATE_CALIB;
  _out_virtuose_status->publish(status);
  // Simulate calibration
  ros::Duration(10.0).sleep();
  // Change state to READY
  status.header.stamp = ros::Time::now();
  status.header.frame_id = "/Calibration";
  status.state = _state = STATE_READY;
  _out_virtuose_status->publish(status);
  // Send response
  response.success = true;
  response.already_calibrated = false;
  response.hardware_down = false;
  response.timeout = false;
  printf("Calibration done\n");
  return true;
}

// Callback for VirtuoseAPI, in admittance mode
void virtAdmittanceCB(VirtContext VC, void *ignored)
{
  // Current date
  uint64_t now = ros::Time::now().toNSec();
  // Physical position
  float disp[7];
  if (!virtGetPhysicalPosition(VC, disp))
  {
    virtuose::out_virtuose_physical_pose ppose;
    ppose.header.stamp = ros::Time::now();
    ppose.virtuose_physical_pose.translation.x = disp[0];
    ppose.virtuose_physical_pose.translation.y = disp[1];
    ppose.virtuose_physical_pose.translation.z = disp[2];
    ppose.virtuose_physical_pose.rotation.x = disp[3];
    ppose.virtuose_physical_pose.rotation.y = disp[4];
    ppose.virtuose_physical_pose.rotation.z = disp[5];
    ppose.virtuose_physical_pose.rotation.w = disp[6];
    _out_virtuose_physical_pose->publish(ppose);
  }
  // Indexed position
  if (!virtGetPosition(VC, disp))
  {
    virtuose::out_virtuose_pose pose;
    pose.header.stamp = ros::Time::now();
    pose.virtuose_pose.translation.x = disp[0];
    pose.virtuose_pose.translation.y = disp[1];
    pose.virtuose_pose.translation.z = disp[2];
    pose.virtuose_pose.rotation.x = disp[3];
    pose.virtuose_pose.rotation.y = disp[4];
    pose.virtuose_pose.rotation.z = disp[5];
    pose.virtuose_pose.rotation.w = disp[6];
    _out_virtuose_pose->publish(pose);
  }
  // Speed
  float speed[6];
  if(!virtGetSpeed(VC, speed))
  {
    virtuose::out_virtuose_speed vspeed;
    vspeed.header.stamp = ros::Time::now();
    vspeed.virtuose_speed.linear.x = speed[0];
    vspeed.virtuose_speed.linear.y = speed[1];
    vspeed.virtuose_speed.linear.z = speed[2];
    vspeed.virtuose_speed.angular.x = speed[3];
    vspeed.virtuose_speed.angular.y = speed[4];
    vspeed.virtuose_speed.angular.z = speed[5];
    _out_virtuose_speed->publish(vspeed);
  }
  // Force
  float force[6];
  if (virtGetForce(VC, force))
  {
    virtuose::out_virtuose_force vforce;
    vforce.header.stamp = ros::Time::now();
    vforce.virtuose_force.force.x = force[0];
    vforce.virtuose_force.force.y = force[1];
    vforce.virtuose_force.force.z = force[2];
    vforce.virtuose_force.torque.x = force[3];
    vforce.virtuose_force.torque.y = force[4];
    vforce.virtuose_force.torque.z = force[5];
    _out_virtuose_force->publish(vforce);
  }
  // Status
  virtuose::out_virtuose_status status;
  status.header.stamp = ros::Time::now();
  status.header.frame_id = "/Admittance";
  status.state = _state = STATE_CART_ADM;
  int button[2] = {0, 0};
  virtGetButton(VC, 0, &button[0]);
  virtGetButton(VC, 1, &button[1]);
  status.buttons = ((button[0] == 1) ? 1 : 0) + ((button[1] == 1) ? 2 : 0);
  int emergency_stop;
  if (virtGetEmergencyStop(VC, &emergency_stop))
  {
    printf("Error in virtGetEmergencyStop with code %d, go to ERROR\n",
        virtGetErrorCode(VC));
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
  status.emergency_stop = (emergency_stop == 1);
  _out_virtuose_status->publish(status);
  // Watchdog on position
  if (((now - start_loop_date) > 30000000) && (now > set_pose_date) && ((now - set_pose_date) > 30000000))
  {
    printf("Timeout on set pose, going to state ERROR\n");
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
  // Set position
  if (virtSetPosition(VC, set_pose))
  {
    printf("Error in virtSetPosition with code %d, go to ERROR\n",
        virtGetErrorCode(VC));
    printf("set_pose was: %f %f %f %f %f %f %f\n",
        set_pose[0], set_pose[1], set_pose[2], 
        set_pose[3], set_pose[4], set_pose[5], set_pose[6]);
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
  // Set speed
  if (virtSetSpeed(VC, set_speed))
  {
    printf("Error in virtSetSpeed with code %d, go to ERROR\n",
       virtGetErrorCode(VC));
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
}

// Callback for service virtuose_admittance
bool admittanceCB(virtuose::virtuose_admittance::Request& request, virtuose::virtuose_admittance::Response &response)
{
  printf("Received admittance request\n");
  // Check state
  if (_state != STATE_READY)
  {
    printf("Not in state READY, go to ERROR\n");
    _state = STATE_ERROR;
    response.success = false;
    response.error = true;
    response.client_id = 0;
    return false;
  }
  // Open connection
  printf("Calling virtOpen with address <%s>\n", request.ip_address.c_str());
  _VC = virtOpen(request.ip_address.c_str());
  if (_VC == NULL)
  {
    printf("Virtuose connection failed with error <%s>, go to ERROR\n", virtGetErrorMessage(0));
    _state = STATE_ERROR;
    response.success = false;
    response.error = true;
    response.client_id = 0;
    return false;
  }
  printf("Virtuose connection OK\n");
  // Set parameters
  int res = 0;
  res += virtSetIndexingMode(_VC, INDEXING_ALL);
  res += virtSetForceFactor(_VC, request.force_factor);
  res += virtSetSpeedFactor(_VC, request.speed_factor);
  res += virtEnableForceFeedback(_VC, request.power_enable);
  res += virtSetPowerOn(_VC, request.power_enable);
  res += virtSetCommandType(_VC, COMMAND_TYPE_VIRTMECH);
  res += virtSaturateTorque(_VC, request.max_force, request.max_torque);
  float base[7];
  base[0] = request.base_frame.translation.x;
  base[1] = request.base_frame.translation.y;
  base[2] = request.base_frame.translation.z;
  base[3] = request.base_frame.rotation.x;
  base[4] = request.base_frame.rotation.y;
  base[5] = request.base_frame.rotation.z;
  base[6] = request.base_frame.rotation.w;
  res += virtSetBaseFrame(_VC, base);
  res += virtSetTimeStep(_VC, 0.001);
  if (res != 0)
  {
    printf("Failed to set parameters with code %d, go to ERROR\n", virtGetErrorCode(_VC));
    virtSetPowerOn(_VC, 0);
    response.success = false;
    response.error = true;
    response.client_id = 0;
    _state = STATE_ERROR;
    return false;
  }
  // Attach callback
  float timeStep = 0.001;
  res += virtSetPeriodicFunction(_VC, virtAdmittanceCB, &timeStep, NULL);
  start_loop_date = ros::Time::now().toNSec();
  res += virtStartLoop(_VC);
  if (res != 0)
  {
    printf("Failed to start virtuose callback with code %d, go to ERROR\n", virtGetErrorCode(_VC));
    virtSetPowerOn(_VC, 0);
    virtStopLoop(_VC);
    response.success = false;
    response.error = true;
    response.client_id = 0;
    _state = STATE_ERROR;
    return false;
  }

  // Set gains
  float K[6], B[6];
  K[0] = K[1] = K[2] = request.Krot;
  K[3] = K[4] = K[5] = request.Ktrans;
  B[0] = B[1] = B[2] = request.Brot;
  B[3] = B[4] = B[5] = request.Btrans;
  if(virtAttachQSVO(_VC, K, B))
  {
    printf("Error in virtAttachQSVO with code %d, go to ERROR\n", virtGetErrorCode(_VC));
    virtSetPowerOn(_VC, 0);
    virtStopLoop(_VC);
    response.success = false;
    response.error = true;
    response.client_id = 0;
    _state = STATE_ERROR;
    return false;
  }

  _state = STATE_CART_ADM;
  response.success = true;
  response.error = false;
  response.client_id = generate_client_id();
  printf("Entering state CART_ADM\n");
  return true;
}

// Callback for VirtuoseAPI, in impedance mode
void virtImpedanceCB(VirtContext VC, void *ignored)
{
  // Current date
  uint64_t now = ros::Time::now().toNSec();
  // Physical position
  float disp[7];
  if (!virtGetPhysicalPosition(VC, disp))
  {
    virtuose::out_virtuose_physical_pose ppose;
    ppose.virtuose_physical_pose.translation.x = disp[0];
    ppose.virtuose_physical_pose.translation.y = disp[1];
    ppose.virtuose_physical_pose.translation.z = disp[2];
    ppose.virtuose_physical_pose.rotation.x = disp[3];
    ppose.virtuose_physical_pose.rotation.y = disp[4];
    ppose.virtuose_physical_pose.rotation.z = disp[5];
    ppose.virtuose_physical_pose.rotation.w = disp[6];
    _out_virtuose_physical_pose->publish(ppose);
  }
  // Indexed position
  if (!virtGetPosition(VC, disp))
  {
    virtuose::out_virtuose_pose pose;
    pose.virtuose_pose.translation.x = disp[0];
    pose.virtuose_pose.translation.y = disp[1];
    pose.virtuose_pose.translation.z = disp[2];
    pose.virtuose_pose.rotation.x = disp[3];
    pose.virtuose_pose.rotation.y = disp[4];
    pose.virtuose_pose.rotation.z = disp[5];
    pose.virtuose_pose.rotation.w = disp[6];
    _out_virtuose_pose->publish(pose);
  }
  // Speed
  float speed[6];
  if (!virtGetSpeed(VC, speed))
  {
    virtuose::out_virtuose_speed vspeed;
    vspeed.virtuose_speed.linear.x = speed[0];
    vspeed.virtuose_speed.linear.y = speed[1];
    vspeed.virtuose_speed.linear.z = speed[2];
    vspeed.virtuose_speed.angular.x = speed[3];
    vspeed.virtuose_speed.angular.y = speed[4];
    vspeed.virtuose_speed.angular.z = speed[5];
    _out_virtuose_speed->publish(vspeed);
  }
  // Status
  virtuose::out_virtuose_status status;
  status.header.stamp = ros::Time::now();
  status.header.frame_id = "/Impedance";
  status.state = _state = STATE_CART_IMP;
  int button[2] = { 0, 0 };
  virtGetButton(VC, 0, &button[0]);
  virtGetButton(VC, 1, &button[1]);
  status.buttons = ((button[0] == 1) ? 1 : 0) + ((button[1] == 1) ? 2 : 0);
  int emergency_stop;
  if (virtGetEmergencyStop(VC, &emergency_stop))
  {
    printf("Error in virtGetEmergencyStop with code %d, go to ERROR\n",
        virtGetErrorCode(VC));
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
  status.emergency_stop = (emergency_stop == 1);
  _out_virtuose_status->publish(status);
  // Watchdog on force
  if (((now - start_loop_date) > 30000000) && (now > set_force_date) && ((now - set_force_date) > 30000000))
  {
    printf("Timeout on force, going to state ERROR\n");
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
  // Set force
  if (virtSetForce(VC, set_force))
  {
    printf("Error in virtSetForce with code %d, go to ERROR\n",
        virtGetErrorCode(VC));
    virtSetPowerOn(VC, 0);
    virtStopLoop(VC);
    _state = STATE_ERROR;
    return;
  }
}

// Callback for service virtuose_impedance
bool impedanceCB(virtuose::virtuose_impedance::Request& request, virtuose::virtuose_impedance::Response &response)
{
  printf("Received impedance request\n");
  // Check state
  if (_state != STATE_READY)
  {
    printf("Not in state READY, go to ERROR\n");
    _state = STATE_ERROR;
    response.success = false;
    response.error = true;
    response.client_id = 0;
    return false;
  }
  // Open connection
  printf("Calling virtOpen with address <%s>\n", request.ip_address.c_str());
  _VC = virtOpen(request.ip_address.c_str());
  if (_VC == NULL)
  {
    printf("Virtuose connection failed with error <%s>, go to ERROR\n", virtGetErrorMessage(0));
    _state = STATE_ERROR;
    response.success = false;
    response.error = true;
    response.client_id = 0;
    return false;
  }
  // Set parameters
  int res = 0;
  res += virtSetIndexingMode(_VC, INDEXING_ALL);
  res += virtSetForceFactor(_VC, request.force_factor);
  res += virtSetSpeedFactor(_VC, request.speed_factor);
  res += virtEnableForceFeedback(_VC, request.power_enable);
  res += virtSetPowerOn(_VC, request.power_enable);
  res += virtSetCommandType(_VC, COMMAND_TYPE_IMPEDANCE);
  res += virtSaturateTorque(_VC, request.max_force, request.max_torque);
  float base[7];
  base[0] = request.base_frame.translation.x;
  base[1] = request.base_frame.translation.y;
  base[2] = request.base_frame.translation.z;
  base[3] = request.base_frame.rotation.x;
  base[4] = request.base_frame.rotation.y;
  base[5] = request.base_frame.rotation.z;
  base[6] = request.base_frame.rotation.w;
  res += virtSetBaseFrame(_VC, base);
  if (res != 0)
  {
    printf("Error setting parameters with code %d, go to ERROR\n", virtGetErrorCode(_VC));
    virtSetPowerOn(_VC, 0);
    response.success = false;
    response.error = true;
    response.client_id = 0;
    _state = STATE_ERROR;
    return false;
  }
  // Attach callback
  float timeStep = 0.001;
  res += virtSetPeriodicFunction(_VC, virtImpedanceCB, &timeStep, NULL);
  start_loop_date = ros::Time::now().toNSec();
  res += virtStartLoop(_VC);
  if (res != 0)
  {
    printf("Failed to start virtuose callback with code %d, go to ERROR\n", virtGetErrorCode(_VC));
    virtSetPowerOn(_VC, 0);
    virtStopLoop(_VC);
    response.success = false;
    response.error = true;
    response.client_id = 0;
    _state = STATE_ERROR;
    return false;
  }

  _state = STATE_CART_IMP;
  response.success = true;
  response.error = false;
  response.client_id = generate_client_id();
  printf("Switching to state CART_IMP\n");
  return true;
}

// Call back for service virtuose_reset
bool resetCB(virtuose::virtuose_reset::Request& request, virtuose::virtuose_reset::Response &response)
{
  printf("Received Reset request\n");
  if (_VC != NULL)
  {
    virtSetPowerOn(_VC, 0);
    virtStopLoop(_VC);
    virtClose(_VC);
    set_pose_date = 0;
    set_speed_date = 0;
    set_force_date = 0;
    _VC = NULL;
  }
  _state = STATE_READY;
  _client_id = 0;
  printf("Switching to state READY\n");
  return true;
}

// Main function
int main(int argc, char **argv)
{
  printf("Starting virtuose node\n");
  // Init ROS library
  ros::init(argc, argv, "virtuose");
  // Create node handle
  ros::NodeHandle n;

  // Create topic publishers
  printf("Creating topics\n");
  ros::Publisher out_virtuose_status = n.advertise<virtuose::out_virtuose_status>("out_virtuose_status", 1);
  _out_virtuose_status = &out_virtuose_status;
  ros::Publisher out_virtuose_pose = n.advertise<virtuose::out_virtuose_pose>("out_virtuose_pose", 1);
  _out_virtuose_pose = &out_virtuose_pose;
  ros::Publisher out_virtuose_physical_pose = n.advertise<virtuose::out_virtuose_physical_pose>("out_virtuose_physical_pose", 1);
  _out_virtuose_physical_pose = &out_virtuose_physical_pose;
  ros::Publisher out_virtuose_speed = n.advertise<virtuose::out_virtuose_speed>("out_virtuose_speed", 1);
  _out_virtuose_speed = &out_virtuose_speed;
  ros::Publisher out_virtuose_force = n.advertise<virtuose::out_virtuose_force>("out_virtuose_force", 1);
  _out_virtuose_force = &out_virtuose_force;
  // Create topic subscribers
  ros::Subscriber in_virtuose_pose = n.subscribe("in_virtuose_pose", 1, in_virtuose_poseCB);
  ros::Subscriber in_virtuose_speed = n.subscribe("in_virtuose_speed", 1, in_virtuose_speedCB);
  ros::Subscriber in_virtuose_force = n.subscribe("in_virtuose_force", 1, in_virtuose_forceCB);

  // Create services
  printf("Creating services\n");
  ros::ServiceServer calibrate = n.advertiseService("virtuose_calibrate", calibrateCB);
  ros::ServiceServer admittance = n.advertiseService("virtuose_admittance", admittanceCB);
  ros::ServiceServer impedance = n.advertiseService("virtuose_impedance", impedanceCB);
  ros::ServiceServer reset = n.advertiseService("virtuose_reset", resetCB);

  // Send first status message
  printf("Publishing initial status\n");
  virtuose::out_virtuose_status status;
  status.header.stamp = ros::Time::now();
  status.header.frame_id = "/Ready";
  status.state = _state = STATE_READY;
  out_virtuose_status.publish(status);

  // Create multithread spinner
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // This loop just to capture ERROR state
  ros::Rate r(100);
  while(ros::ok())
  {
    if (_state == STATE_ERROR)
    {
      if (_VC != NULL)
      {
        virtStopLoop(_VC);
        ros::Duration(0.1).sleep();
        virtClose(_VC);
        _VC = NULL;
        status.header.stamp = ros::Time::now();
        status.header.frame_id = "/Error";
        status.state = _state = STATE_ERROR;
        out_virtuose_status.publish(status);
        printf("Entered state ERROR\n");
      }
    }
    r.sleep();
  }

  return 0;
}
