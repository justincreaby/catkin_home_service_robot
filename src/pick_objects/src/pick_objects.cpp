#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //set up publisher to broadcast if robot is at goal marker
  ros::NodeHandle n;
  ros::Publisher goal_reach_pub = n.advertise<std_msgs::UInt8>("/goal_reached", 1);


  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;
  std_msgs::UInt8 status_msg;  // goal reach status

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -8;
  goal.target_pose.pose.position.y = 1.5;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, we've reached the pickup zone");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");

  ROS_INFO("Waiting 5 seconds at the pickup zone");
  ros::Duration(5.0).sleep();
  ROS_INFO("Done waiting, moving to drop off zone");
  status_msg.data = 1;
  goal_reach_pub.publish(status_msg); // pickup

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -1.5;
  goal.target_pose.pose.position.y = -7.0;
  goal.target_pose.pose.orientation.z = -0.707;
  goal.target_pose.pose.orientation.w = -0.707;

 // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal for drop off");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, we've reached the drop off zone");
  else
    ROS_INFO("The base failed to move forward 1 meter for some reason");
  
  status_msg.data = 2;
  goal_reach_pub.publish(status_msg); // dropoff
  ROS_INFO("Waiting 5 seconds at the drop off zone");
  ros::Duration(5.0).sleep();
  ROS_INFO("Done");
}
