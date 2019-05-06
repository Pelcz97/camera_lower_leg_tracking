#include "ros/ros.h" 

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#define GND_LEVEL (0.3)


ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_cloud)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  
  // Do some Data Processing here
  //TODO Data Processing
  output = *input_cloud;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "remove_gnd");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("gnd_removed", 1);

  // Spin
  ros::spin ();
}
