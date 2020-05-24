#include <ros/ros.h>
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <sstream>

// define a globalc client that can request services
ros::ServiceClient client;

// this functions calls the command_robot service to drive the robot in
// the specified direction
void drive_bot(float linear_x, float angular_z)
{
  // Request a service and pass the velocities to it to drive the robot
  
  ROS_INFO_STREAM("Driving bot");
  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = linear_x;
  srv.request.angular_z = angular_z;

  if (not client.call(srv))
  {
    ROS_ERROR("Failed to call service drive to target");
  }

}

// this callback functions continuosly executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
  const int white_pixel = 255;
  /*  
  ROS_INFO_STREAM("image height" + std::to_string(img.height));
  ROS_INFO_STREAM("image step" + std::to_string(img.step));
  */

  // TODO: Loop through each pixel in the image and check if there's 
  // a bright white one. Then, identify if this pixel falls in the left,
  // mid, or right side of the image
  // Depending on the white ball position, call the drive_bot function and 
  // pass velocities to it
  // Request a stop when there's no white ball seen by the camera

  const int image_slice_width = img.step / 3;

  ROS_INFO_STREAM("image_slice_width =" + std::to_string(image_slice_width));

  int j;
  bool found = false;

  // for (int i = 0; i < img.height; i++) 
  for (int i = 0; not found and i < img.height; i++) 
  {
    // for (j = 0; j < img.step; j++)
    for (j; j < img.step-3; j += 3)
    {
      // std::stringstream ss;
      // ss << "i=" << i << " j=" << j << " pixel=" << std::to_string(img.data[i*img.step + (j + 0)]);
      // ROS_INFO_STREAM(ss.str());
      //
      
      // ROS_INFO_STREAM("i=" + i + std::to_string(" j=") + j + std::to_string(" pixel=") + std::to_string(img.data[i*img.step + (j + 0)]));
      // ROS_INFO_STREAM("img.data[i*img.step + (j + 0)]" + std::to_string(img.data[i*img.step + (j + 0)]));
      // if (img.data[i*img.step + j] == white_pixel) {
      //   ROS_INFO_STREAM("white pixel = "+ std::to_string(img.data[i*img.step + j]));
      // }
      if (img.data[i*img.step + j] == white_pixel)
      {
    
        ROS_INFO_STREAM("img.data[i*img.step + (j + 0)]" + std::to_string(img.data[i*img.step + (j + 0)]));
        ROS_INFO_STREAM("img.data[i*img.step + (j + 1)]" + std::to_string(img.data[i*img.step + (j + 1)]));
        ROS_INFO_STREAM("img.data[i*img.step + (j + 2)]" + std::to_string(img.data[i*img.step + (j + 2)]));
      }

      // img.data only has one index
      if (img.data[i*img.step +  j     ] == white_pixel and
          img.data[i*img.step + (j + 1)] == white_pixel and
          img.data[i*img.step + (j + 2)] == white_pixel)
      {
          found = true;
          ROS_INFO_STREAM("found");
          break;
      }
     
    }
  }

  if (found)
  {
    // determine which third of image ball is
    switch (j / image_slice_width) // split into 3
    {
      case 0: // white ball to the left
        drive_bot(0.5, 0.5);
        break;
      case 1: // white ball in front
        drive_bot(0.5, 0.0);
        break;
      default: // white ball to the right 
        drive_bot(0.5, -0.5);
        break;
    }
  }
  else
  {
    // no white pixel seen so stop the bot
    drive_bot(0.0, 0.0);

  }

}

int main (int argc, char** argv)
{
  // initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");
  ros::NodeHandle node_handle;

  // define a client service capable of requesting services from command_robot
  client = node_handle.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

  // subscribe to /camera/rgb/image_raw to read image date in the 
  // process_image_callback function
  ros::Subscriber sub1 = node_handle.subscribe("/camera/rgb/image_raw", 2, process_image_callback);

  // handle ROS communication events
  ros::spin();

  return EXIT_SUCCESS;
}
