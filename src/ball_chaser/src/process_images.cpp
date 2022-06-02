#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot

    //ROS_INFO_STREAM("Drive the robot");

    // Request centered joint angles [1.57, 1.57]
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the safe_move service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service ball_chaser");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    bool has_white_pixel = false;
    //ROS_INFO("height = %d, width = %d, step = %d", img.height, img.width, img.step);
  
    int left_white_pixel_position = img.step; // max
    int right_white_pixel_position = 0; // min
    int pixelIndex = 0;
    int ball_position = -1;
    int ball_size = 0;
    // Loop through each pixel in the image and check if its equal to the first one
    for (int i = 0; i < img.height; i++) {
      for (int j = 0; j < img.step; j=j+3) {

        //ROS_INFO("pixelIndex = %d: i, j = %d, %d", pixelIndex, i, j);
        //ROS_INFO("img.data[pixelIndex] = %d", img.data[pixelIndex]);
        
        if (   img.data[pixelIndex] == white_pixel
            && img.data[pixelIndex + 1] == white_pixel
            && img.data[pixelIndex + 1] == white_pixel) {
            //ROS_INFO("img.data[i] = %d", img.data[i]);
            has_white_pixel = true;
            left_white_pixel_position = std::min(left_white_pixel_position, j);
            right_white_pixel_position = std::max(right_white_pixel_position, j);
            ball_size = right_white_pixel_position - left_white_pixel_position;
            ball_position = left_white_pixel_position + (int)(0.5 * ball_size);
        }
        pixelIndex = pixelIndex + 3;
      }
    }

    if (has_white_pixel)
    {
      ROS_INFO("Has Ball: leftSide = %d, rightSide = %d, size = %d, pos = %d", left_white_pixel_position, right_white_pixel_position, ball_size, ball_position);
      
      float speed = 0.0;
      float turnDir = 0.0;
      if (ball_size > 800)
      {
        speed = -0.25;
      }
      else if (ball_size < 600)
      {
        speed = 0.25;
      }
      else
      {
        speed = 0.0;
      }

      if (ball_position > 1300)
      {
        turnDir = -1.0;
      }
      else if (ball_position < 1100)
      {
        turnDir = 1.0;
      }
      else
      {
        turnDir = 0.0;
      }


      drive_robot(speed, turnDir);
    }
    else
    {
      ROS_INFO("No Ball");
      drive_robot(0.0, 0.0);
    }
    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}