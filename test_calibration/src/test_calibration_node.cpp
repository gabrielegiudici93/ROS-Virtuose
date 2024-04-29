#include "ros/ros.h"
#include "std_msgs/String.h"
#include "virtuose/out_virtuose_status.h"
#include "virtuose/virtuose_calibrate.h"
#include "virtuose/virtuose_reset.h"

// Storage for virtuose_node status
uint64_t status_date;
int status_state = 0;
int status_button;

// Callback for topic out_virtuose_status
void out_virtuose_statusCB(const virtuose::out_virtuose_status::ConstPtr& msg)
{
  // Store virtuose_node status
  status_date = msg->header.stamp.toNSec();
  status_state = msg->state;
  status_button = msg->buttons;
  if (status_state == 3)
    printf("Please push the force-feedback button!\n");
}

// Main function
int main(int argc, char **argv)
{
  printf("Starting test_calibration node\n");
  // Init ROS library
  ros::init(argc, argv, "test_calibration");
  // Create node handle
  ros::NodeHandle n;

  // Connect to virtuose_node
  printf("Connecting to virtuose node\n");
  ros::Subscriber out_virtuose_status = n.subscribe<virtuose::out_virtuose_status>("out_virtuose_status", 1, out_virtuose_statusCB);
  ros::ServiceClient virtuose_calibrate = n.serviceClient<virtuose::virtuose_calibrate>("virtuose_calibrate");
  ros::ServiceClient virtuose_reset = n.serviceClient<virtuose::virtuose_reset>("virtuose_reset");

  // Reset virtuose_node
  virtuose::virtuose_reset res;
  virtuose_reset.call(res);
  
  // Create multithread spinner
  ros::AsyncSpinner spinner(1);
  spinner.start();

  uint64_t start = ros::Time::now().toNSec();

  // Send request for automatic calibration
  printf("Sending calibration request\n");
  virtuose::virtuose_calibrate cal;
  cal.request.ip_address = "127.0.0.1";
  cal.request.automatic = true;
  if (!virtuose_calibrate.call(cal))
  {
    printf("Call failed\n");
    return 0;
  }

  // Wait for state to switch back to READY
  ros::Rate r(10);
  int ctr = 0;
  while(ros::ok() && (status_state != 1))
  {
    ctr ++;
    if (ctr % 10 == 0)
    {
      printf("Status: %ld %d\n", status_date - start, status_state);
    }
    r.sleep();
  }

  // Reset virtuose_node
  virtuose_reset.call(res);
  printf("Calibration done\n");

  return 0;
}
