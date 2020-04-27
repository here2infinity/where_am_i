#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
//Include the ball_chaser "DriveToTarget" header file
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands
ros::Publisher motor_command_publisher;

bool handle_drive_request(
    ball_chaser::DriveToTarget::Request & req,
    ball_chaser::DriveToTarget::Response & res)
{
  ROS_INFO("DriveToTarget received: \nlinear_x=%1.2f\n angular_z=%1.2f",
      (float) req.linear_x, req.angular_z);
 
  // Create a motor_command object of type geometry_msgs::Twist
  geometry_msgs::Twist motor_command;

  // set wheel velocities
  motor_command.linear.x = req.linear_x;
  motor_command.angular.z = req.angular_z;

  // publish angles to drive the bot
  motor_command_publisher.publish(motor_command);

  res.msg_feedback = "Drive parameters sent: " + "\n" + "linear z = " + std::to_string(motor_command.linear.x) + "\n" + "angular z = " + std::to_string(motor_command.angular.z);
  
  ROS_INFO_STREAM(res.msg_feedback);

  return true;
}

int main(int argc, char** argv) 
{
  // initialize a ROS node
  ros::init(argc, argv, "drive_bot");

  // create a ROS node handle object
  ros::NodeHandle node_handle;

  // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
  motor_command_publisher = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

  // Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
  ros::ServiceServer service = node_handle.advertiseService("/ball_chaser/command_robot", handle_drive_request);

  ros::spin();

  return EXIT_SUCCESS;
}

