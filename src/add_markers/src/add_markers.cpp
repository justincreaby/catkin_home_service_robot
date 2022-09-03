#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>


/* robot goal proximity callback function */
uint8_t goal_reach_state = 0;
void goalReachCallback(const std_msgs::UInt8::ConstPtr& msg)
{
   ROS_INFO("[goal_reach_State updated");
   goal_reach_state = msg->data;
   return;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(5);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_sub = n.subscribe("/goal_reached", 1, goalReachCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    ros::spinOnce();
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_marker";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    if (goal_reach_state == 0)
    {
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -8.0;
    marker.pose.position.y = 1.5;
    }
    else if (goal_reach_state == 1)
    {
      marker.action = visualization_msgs::Marker::DELETE;
      marker.pose.position.x = -8.0;
      marker.pose.position.y = 1.5;
    }
    else
    {
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = -1.5;
      marker.pose.position.y = -7.0;
    }

    

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker.lifetime = ros::Duration();
    marker_pub.publish(marker);
    

    // Cycle between different shapes
    //switch (shape)
    //{
    //case visualization_msgs::Marker::CUBE:
    //  shape = visualization_msgs::Marker::SPHERE;
    //  break;
    //case visualization_msgs::Marker::SPHERE:
    //  shape = visualization_msgs::Marker::ARROW;
    //  break;
    //case visualization_msgs::Marker::ARROW:
    //  shape = visualization_msgs::Marker::CYLINDER;
    //  break;
    //case visualization_msgs::Marker::CYLINDER:
    //  shape = visualization_msgs::Marker::CUBE;
    //  break;
    //}

    //ROS_INFO("[goal_reach_State: %d", goal_reach_state);
    r.sleep();
  }
}
