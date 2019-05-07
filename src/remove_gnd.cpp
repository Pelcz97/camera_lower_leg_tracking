#include "ros/ros.h" 
#include "../include/pcl_types.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/projection_matrix.h>
#include "pcl_ros/point_cloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/convert.h>


#define GND_LEVEL (0.03)


ros::Publisher pub;

void cloud_cb (const Cloud_cptr& input_cloud)
{    
    /*
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    sensor_msgs::PointCloud2 cloud_in_tr;
    
    geometry_msgs::TransformStamped transformStamped;
    
    
    try {
        transformStamped = tfBuffer.lookupTransform("base_link", "camera_link", ros::Time(0));
        std::cout << transformStamped << std::endl;
        tf2::doTransform(input_cloud, cloud_in_tr, transformStamped);
    }
    catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
    }    
    */
    Indices object_indices;
    for( size_t i = 0; i < input_cloud->size(); i++ ){
    	float z = input_cloud->points[i].z;
        if( z > GND_LEVEL) {
            object_indices.push_back(i);
        }
    }
    Cloud output(*input_cloud, object_indices );

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
