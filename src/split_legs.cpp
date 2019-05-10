#include "../include/pcl_types.h"
#include "ros/ros.h" 

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include "pcl_ros/point_cloud.h"


ros::Publisher pub1, pub2;

void cloud_cb (const Cloud_cptr& input_cloud) {
    


    
  ROS_INFO("PointCloud before filtering has: %lu data points", input_cloud->points.size());
  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<Point> vg;
  Cloud_ptr cloud_filtered (new Cloud);
  vg.setInputCloud (input_cloud);
  vg.setLeafSize (0.01f, 0.01f, 0.01f);
  vg.filter (*cloud_filtered);
  ROS_INFO("PointCloud after filtering has: %lu data points", cloud_filtered->points.size());
  
//     Cloud_cptr cloud_filtered = input_cloud;

  if (cloud_filtered->empty()) {
      ROS_INFO("NO CLUSTER FOUND");
      return;
  }
  
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
  tree->setInputCloud (cloud_filtered);

  //Setting the parameters for cluster extraction
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<Point> ec;
  ec.setClusterTolerance (0.05); // 5cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (2000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud_filtered);
  ec.extract (cluster_indices);
  
  
  ROS_INFO("Cluster Count: %lu", cluster_indices.size());
  for (int i = 0; i < cluster_indices.size(); i++) {
      ROS_INFO("Cluster %d has %lu points", i, cluster_indices.at(i).indices.size());
  }
  
  
  std::vector<Cloud> clusters;
  //Creating PointClouds for each cluster. clusters is sorted by the size of the cluster.
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    Cloud cloud_cluster(*cloud_filtered, it->indices );
    clusters.push_back(cloud_cluster);
  }
  
  if (clusters.size() == 1) {
  Cloud fst_leg = clusters.at(0);
  
  ROS_INFO("One Cluster published");
  pub1.publish(fst_leg);
  }
  else if (clusters.size() == 2) {
  Cloud fst_leg = clusters.at(0);
  Cloud snd_leg = clusters.at(1);
  
  pub1.publish(fst_leg);
  pub2.publish(snd_leg);
  }
  else {
  ROS_INFO("More than 2 Clusters detected. Only the 2 largest will be published.");
  Cloud fst_leg = clusters.at(0);
  Cloud snd_leg = clusters.at(1);
  
  pub1.publish(fst_leg);
  pub2.publish(snd_leg);
  }

}


int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "split_legs");
  ros::NodeHandle nh;
  
  ROS_INFO("Started");

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("gnd_removed", 1000, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub1 = nh.advertise<sensor_msgs::PointCloud2> ("fst_leg", 1);
  pub2 = nh.advertise<sensor_msgs::PointCloud2> ("snd_leg", 1);

  // Spin
  ros::spin ();
}
