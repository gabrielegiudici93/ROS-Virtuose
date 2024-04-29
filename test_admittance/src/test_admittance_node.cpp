#include "ros/ros.h"
#include "std_msgs/String.h"
#include "virtuose/out_virtuose_status.h"
#include "virtuose/out_virtuose_pose.h"
#include "virtuose/out_virtuose_speed.h"
#include "virtuose/in_virtuose_pose.h"
#include "virtuose/virtuose_calibrate.h"
#include "virtuose/virtuose_admittance.h"
#include "virtuose/virtuose_reset.h"

// Storage for virtuose pose
float cur_pose[7];
uint64_t pose_date;

// Storage for virtuose_node status
uint64_t status_date;
int status_state = 0;
int status_button;

// Callback for topic out_virtuose_pose
void out_virtuose_poseCB(const virtuose::out_virtuose_pose::ConstPtr& msg)
{
  // Store last pose date
  pose_date = msg->header.stamp.toNSec();
  cur_pose[0] = msg->virtuose_pose.translation.x;
  cur_pose[1] = msg->virtuose_pose.translation.y;
  cur_pose[2] = msg->virtuose_pose.translation.z;
  cur_pose[3] = msg->virtuose_pose.rotation.x;
  cur_pose[4] = msg->virtuose_pose.rotation.y;
  cur_pose[5] = msg->virtuose_pose.rotation.z;
  cur_pose[6] = msg->virtuose_pose.rotation.w;
}

// Callback for topic out_virtuose_status
void out_virtuose_statusCB(const virtuose::out_virtuose_status::ConstPtr& msg)
{
  // Store last status date
  status_date = msg->header.stamp.toNSec();
  status_state = msg->state;
  status_button = msg->buttons;
}

// Main function
int main(int argc, char **argv)
{
  printf("Starting test_admittance node\n");
  // Init ROS library
  ros::init(argc, argv, "test_admittance");
  // Create node handle
  ros::NodeHandle n;

  // Connect to virtuose_node
  printf("Connecting to virtuose node\n");
  ros::Subscriber out_virtuose_status = n.subscribe<virtuose::out_virtuose_status>("out_virtuose_status", 1, out_virtuose_statusCB);
  ros::Subscriber out_virtuose_pose = n.subscribe<virtuose::out_virtuose_pose>("out_virtuose_pose", 1, out_virtuose_poseCB);
  ros::Publisher in_virtuose_pose = n.advertise<virtuose::in_virtuose_pose>("in_virtuose_pose", 1);
  ros::ServiceClient virtuose_admittance = n.serviceClient<virtuose::virtuose_admittance>("virtuose_admittance");
  ros::ServiceClient virtuose_reset = n.serviceClient<virtuose::virtuose_reset>("virtuose_reset");

  // Reset virtuose_node
  virtuose::virtuose_reset res;
  virtuose_reset.call(res);
  
  // Create multithread spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  uint64_t start = ros::Time::now().toNSec();

  // Request admittance mode
  printf("Sending admittance request\n");
  virtuose::virtuose_admittance adm;
  adm.request.ip_address = "localhost#53210";
  adm.request.indexing_mode = 0;
  adm.request.speed_factor = 1.0;
  adm.request.force_factor = 1.0;
  adm.request.power_enable = 1;
  adm.request.max_force = 10.0;
  adm.request.max_torque = 1.0;
  adm.request.Ktrans = 2000.0;
  adm.request.Btrans = 20.0;
  adm.request.Krot = 5.0;
  adm.request.Brot = 0.25;
  adm.request.base_frame.translation.x = 0.0;
  adm.request.base_frame.translation.y = 0.0;
  adm.request.base_frame.translation.z = 0.0;
  adm.request.base_frame.rotation.x = 0.0;
  adm.request.base_frame.rotation.y = 0.0;
  adm.request.base_frame.rotation.z = 0.0;
  adm.request.base_frame.rotation.w = 1.0;
  if (!virtuose_admittance.call(adm))
  {
    printf("Call failed\n");
    return 0;
  }
  // Store client ID given by virtuose_node
  uint32_t client_id = 0;
  client_id = adm.response.client_id;
  printf("Our client ID is %d\n", client_id);

  // Perfom admittance control at 500 Hz
  ros::Rate r(500);
  int ctr = 0;
  while(ros::ok())
  {
    // Copy back current pose
    virtuose::in_virtuose_pose pose;
    pose.header.stamp = ros::Time::now();
    pose.client_id = client_id;
    pose.virtuose_pose.translation.x = cur_pose[0];
    pose.virtuose_pose.translation.y = cur_pose[1];
    pose.virtuose_pose.translation.z = cur_pose[2];
    pose.virtuose_pose.rotation.x = cur_pose[3];
    pose.virtuose_pose.rotation.y = cur_pose[4];
    pose.virtuose_pose.rotation.z = cur_pose[5];
    pose.virtuose_pose.rotation.w = cur_pose[6];
    // Simulate virtual wall at Z=-10 cm
    if (pose.virtuose_pose.translation.z < -0.1)
      pose.virtuose_pose.translation.z = -0.1;
    in_virtuose_pose.publish(pose);
    ctr ++;
    // Print status every second
    if (ctr % 500 == 0)
    {
      printf("Status: %ld %d %d %f %f %f\n", status_date - start, status_state, status_button,
             cur_pose[0], cur_pose[1], cur_pose[2]);
    }
    r.sleep();
  }

  // Reset virtuose_node
  virtuose_reset.call(res);

  return 0;
}
