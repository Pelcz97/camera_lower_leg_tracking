#include "ros/ros.h" 
#include "../include/pcl_types.h"

#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"
#include <pcl/filters/voxel_grid.h>

#include <geometry_msgs/TransformStamped.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>


#define GND_LEVEL (0.03)


ros::Publisher pub;
geometry_msgs::TransformStamped transformStamped;


void cloud_cb (const Cloud_cptr& input_cloud)
{    
    
    
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    ROS_INFO("PointCloud before filtering has: %lu data points", input_cloud->points.size());
    pcl::VoxelGrid<Point> vg;
    Cloud_ptr cloud_filtered (new Cloud);
    vg.setInputCloud (input_cloud);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    ROS_INFO("PointCloud before filtering has: %lu data points", cloud_filtered->points.size());
    
    Cloud input_cloud_tr = *cloud_filtered;
    sensor_msgs::PointCloud2 sens_msg_input_cloud, sens_msg_input_cloud_tr;
    
    ROS_INFO("Tranform frame is %s und %s", transformStamped.header.frame_id.c_str(), transformStamped.child_frame_id.c_str());
    
    //Conversion from Cloud to sensor_msgs
    pcl::toROSMsg(*input_cloud, sens_msg_input_cloud);     
    
    //Transformation
    tf2::doTransform(sens_msg_input_cloud, sens_msg_input_cloud_tr, transformStamped);

    //Conversion from sensor_msgs::PointCloud2 to Cloud
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(sens_msg_input_cloud_tr,pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, input_cloud_tr);
    
    
    //Delete the GroundPoints
    Indices object_indices;
    for( size_t i = 0; i < input_cloud_tr.size(); i++ ){
    	float z = input_cloud_tr.points[i].z;
        if( z > GND_LEVEL) {
            object_indices.push_back(i);
        }
    }
    Cloud output(input_cloud_tr, object_indices );

  // Publish the data.
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "remove_gnd");
  ros::NodeHandle nh;
  
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  
  
  try {
     transformStamped = tfBuffer.lookupTransform("base_link", "camera_depth_optical_frame", ros::Time(0), ros::Duration(2));
  } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
  }

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("gnd_removed", 1);

  // Spin
  ros::spin ();
}
