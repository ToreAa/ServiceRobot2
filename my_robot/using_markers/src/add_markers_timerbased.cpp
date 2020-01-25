#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

double posx;
double posy;
double goalx1 = -5.6;
double goaly1 = 7.4;
double goalx2 = -2;
double goaly2 = 4;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  posx = msg->pose.pose.position.x;
  posy = msg->pose.pose.position.y;
  ROS_INFO("Position-> x: [%f], y: [%f]", posx , posy);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  goalx1 = msg->pose.position.x;
  goaly1 = msg->pose.position.y;
  ROS_INFO("Position-> x: [%f], y: [%f]", posx , posy);
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  int i = 0; //integer for stepping through state-machine

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    ros::Subscriber sub_odom = n.subscribe("/odom", 100, odomCallback);
    ros::Subscriber sub_goal = n.subscribe("/move_base_simple/goal", 100, poseCallback);
    
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type to CUBE and the orientation 
    marker.type = visualization_msgs::Marker::CUBE;

    // State-machine for the various steps
    switch (i)
    {
    case 0: // Publish the marker at the pickup zone for 5 seconds
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = goalx1;
      marker.pose.position.y = goaly1;
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      marker.color.r = 0.9f;
      marker.color.g = 0.2f;
      marker.color.b = 0.5f;
      marker.color.a = 0.8;
      marker.lifetime = ros::Duration(); 
      marker_pub.publish(marker); 
      ros::Duration(2.0).sleep();
      i = 1;
      break;
    case 1: // Wait for 5 seconds
      marker.action = visualization_msgs::Marker::DELETEALL; 
      marker_pub.publish(marker);
      ros::Duration(2.0).sleep();
      i = 2;
      break;
    case 2: // Publish the marker at the drop off zone  
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = goalx2;
      marker.pose.position.y = goaly2;
      marker.scale.x = 0.3;
      marker.scale.y = 0.3;
      marker.scale.z = 0.3;
      marker.color.r = 0.9f;
      marker.color.g = 0.2f;
      marker.color.b = 0.5f;
      marker.color.a = 0.8;
      marker.lifetime = ros::Duration();
      marker_pub.publish(marker);
      ros::Duration(3.0).sleep();
      i = 0; 
      break;
    } 

    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    ros::spinOnce();
    r.sleep();
  }
}
