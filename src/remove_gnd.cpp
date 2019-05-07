#include "ros/ros.h" 

#include <sensor_msgs/PointCloud2.h>
 #include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#define GND_LEVEL (0.3)


ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
    pcl::PointCloud<pcl::PointXYZRGB> output;
    
    pcl::PCLPointCloud2 input;
    pcl_conversions::toPCL(*input_msg,input);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_pt_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(input,*input_pt_cloud);
    

  ROS_INFO("PCL RECEIVED");
  // Do some Data Processing here
  
    
  
  std::vector<int> indices;
  for( size_t i = 0; i < input_pt_cloud->points.size(); i++ ){
    	float z = input_pt_cloud->points[i].z;
        if( z > GND_LEVEL ){
            indices.push_back(i);
        }
    }
 
  copyPointCloud(*input_pt_cloud, output);
    
  // Publish the data.
  pub.publish (output);
}

int main (int argc, char** argv)
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
