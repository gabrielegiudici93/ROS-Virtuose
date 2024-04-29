#include "ros/ros.h"
#include "std_msgs/String.h"
#include "virtuose/out_virtuose_status.h"
#include "virtuose/out_virtuose_pose.h"
#include "virtuose/in_virtuose_force.h"
#include "virtuose/virtuose_impedance.h"
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
  printf("Starting test_impedance node\n");
  // Init ROS library
  ros::init(argc, argv, "test_impedance");
  // Create node handle
  ros::NodeHandle n;

  // Connect to virtuose_node
  printf("Connecting to virtuose node\n");
  ros::Subscriber out_virtuose_status = n.subscribe<virtuose::out_virtuose_status>("out_virtuose_status", 1, out_virtuose_statusCB);
  ros::Subscriber out_virtuose_pose = n.subscribe<virtuose::out_virtuose_pose>("out_virtuose_pose", 1, out_virtuose_poseCB);
  ros::Publisher in_virtuose_force = n.advertise<virtuose::in_virtuose_force>("in_virtuose_force", 1);
  ros::ServiceClient virtuose_impedance = n.serviceClient<virtuose::virtuose_impedance>("virtuose_impedance");
  ros::ServiceClient virtuose_reset = n.serviceClient<virtuose::virtuose_reset>("virtuose_reset");

  // Reset virtuose_node
  virtuose::virtuose_reset res;
  virtuose_reset.call(res);
  
  // Create multithread spinner
  ros::AsyncSpinner spinner(2);
  spinner.start();

  uint64_t start = ros::Time::now().toNSec();

  // Request impedance mode
  printf("Sending impedance request\n");
  virtuose::virtuose_impedance imp;
  imp.request.ip_address = "localhost#53210";
  imp.request.indexing_mode = 0;
  imp.request.speed_factor = 1.0;
  imp.request.force_factor = 1.0;
  imp.request.power_enable = 1;
  imp.request.max_force = 10.0;
  imp.request.max_torque = 1.0;
  imp.request.base_frame.translation.x = 0.0;
  imp.request.base_frame.translation.y = 0.0;
  imp.request.base_frame.translation.z = 0.0;
  imp.request.base_frame.rotation.x = 0.0;
  imp.request.base_frame.rotation.y = 0.0;
  imp.request.base_frame.rotation.z = 0.0;
  imp.request.base_frame.rotation.w = 1.0;
  if (!virtuose_impedance.call(imp))
  {
    printf("Call failed\n");
    return 0;
  }
  // Store client ID given by virtuose_node
  uint32_t client_id = 0;
  client_id = imp.response.client_id;
  printf("Our client ID is %d\n", client_id);

  // Perform impedance loop at 500 Hz
  ros::Rate r(500);
  int ctr = 0;
  while(ros::ok())
  {
    // Create null force
    virtuose::in_virtuose_force force;
    force.header.stamp = ros::Time::now();
    force.client_id = client_id;
    force.virtuose_force.force.x = 0.0;
    force.virtuose_force.force.y = 0.0;
    force.virtuose_force.force.z = 0.0;
    force.virtuose_force.torque.x = 0.0;
    force.virtuose_force.torque.y = 0.0;
    force.virtuose_force.torque.z = 0.0;
    // Simulate virtual wall at z=-10 cm
    if (cur_pose[2] < -0.1)
    {
      force.virtuose_force.force.z = - (cur_pose[2] + 0.1) * 1000.0;
      if (force.virtuose_force.force.z > 30.0)
        force.virtuose_force.force.z = 30.0;
    }
    in_virtuose_force.publish(force);
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
