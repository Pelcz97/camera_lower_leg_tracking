#include "../include/pcl_types.h"
#include "ros/ros.h" 

#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>


#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>


#include "pcl_ros/point_cloud.h"





ros::Publisher pub1, pub2;

void cloud_cb1 (const Cloud_cptr& input_cloud) {
    
    geometry_msgs::PointStamped maxX;
    maxX.header.stamp = ros::Time::now();
    maxX.header.frame_id = "base_link";
    maxX.header.seq++;
    maxX.point.x = input_cloud->points[0].x;
    maxX.point.y = input_cloud->points[0].y;
    maxX.point.z = input_cloud->points[0].z;

    for( size_t i = 0; i < input_cloud->size(); i++ ){
    	float X = input_cloud->points[i].x;
        if( X > maxX.point.x) {
            maxX.point.x = input_cloud->points[i].x;
            maxX.point.y = input_cloud->points[i].y;
            maxX.point.z = input_cloud->points[i].z;
        }
    }
    ROS_INFO("LEFT_TOE has been published");
    pub1.publish(maxX);
}

void cloud_cb2 (const Cloud_cptr& input_cloud) {
    
    geometry_msgs::PointStamped maxX;
    maxX.header.stamp = ros::Time::now();
    maxX.header.frame_id = "base_link";
    maxX.header.seq++;
    maxX.point.x = input_cloud->points[0].x;
    maxX.point.y = input_cloud->points[0].y;
    maxX.point.z = input_cloud->points[0].z;

    for( size_t i = 0; i < input_cloud->size(); i++ ){
    	float X = input_cloud->points[i].x;
        if( X > maxX.point.x) {
            maxX.point.x = input_cloud->points[i].x;
            maxX.point.y = input_cloud->points[i].y;
            maxX.point.z = input_cloud->points[i].z;;
        }
    }
    ROS_INFO("RIGHT_TOE has been published");
    pub2.publish(maxX);
}



int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "find_toes");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub1 = nh.subscribe ("left_leg", 1, cloud_cb1);
  ros::Subscriber sub2 = nh.subscribe ("right_leg", 1, cloud_cb2);


  // Create a ROS publisher for the output point cloud
  pub1 = nh.advertise<geometry_msgs::PointStamped> ("left_toe", 1);
  pub2= nh.advertise<geometry_msgs::PointStamped> ("right_toe", 1);

  // Spin
  ros::spin ();
}
 
