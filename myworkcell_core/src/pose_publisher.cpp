#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher visual_pub;
ros::Publisher pose_pub;

static std::string& camera_frame_name()
{
  static std::string camera_frame;
  return camera_frame;
} 

// Singleton Instance of Object Position
static geometry_msgs::Pose& pose()
{
  static geometry_msgs::Pose pose;
  return pose;
}

// Given a marker w/ pose data, publish an RViz visualization
// You'll need to add a "Marker" visualizer in RVIZ AND define
// the "camera_frame" TF frame somewhere to see it.
static void pubVisualMarker(const visualization_msgs::Marker& m)
{
  const double width = 0.08;
  const double thickness = 0.005;
  ros::Time curr_stamp(ros::Time::now());
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = m.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "ar_marker_visual";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = m.pose;
  marker.pose.position.z -= thickness / 2.0;
  marker.scale.x = width;
  marker.scale.y = width;
  marker.scale.z = thickness;
  marker.color.a = 1.0;
  marker.color.b = 1.0;
  
  geometry_msgs::PoseStamped poseMsg;
  poseMsg.header.frame_id = "camera_frame";
  poseMsg.header.stamp = curr_stamp;
  poseMsg.pose=m.pose;

  pose_pub.publish(poseMsg);
  visual_pub.publish(marker);
}

void pubCallback(const ros::TimerEvent&)
{
  geometry_msgs::Pose p = pose();
  visualization_msgs::Marker m;
  m.header.frame_id = camera_frame_name();
  m.header.stamp = ros::Time::now();
  m.pose = p;

  visual_pub.publish(m);
  
  pubVisualMarker(m); // visualize the marker
}

int main(int argc, char **argv)
{
  // Set up ROS.
  ros::init(argc, argv, "pose_publisher");
  ros::NodeHandle nh, pnh ("~");
  visual_pub = nh.advertise<visualization_msgs::Marker>("/aruco_single/marker", 1);
  pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/aruco_single/pose", 100);

  // init pose
  pose().orientation.w = 1.0; // facing straight up
  pnh.param<double>("x_pos", pose().position.x, -0.1);
  pnh.param<double>("y_pos", pose().position.y, -0.3);
  pnh.param<double>("z_pos", pose().position.z, 0.5);
  
  pnh.param<std::string>("camera_frame", camera_frame_name(), "camera_frame");

  ROS_INFO("Starting simulated ARMarker publisher");  
  ros::Timer t = nh.createTimer(ros::Duration(0.1), pubCallback, false, true);
  ros::spin();
}
