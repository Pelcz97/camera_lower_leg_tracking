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


#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#define GND_LEVEL (0.03)

geometry_msgs::TransformStamped transformStamped;
ros::Publisher pub_left_leg, pub_right_leg;



Cloud removeGround(sensor_msgs::PointCloud2 input_cloud) {
    ROS_INFO("Tranform frame is %s und %s", transformStamped.header.frame_id.c_str(), transformStamped.child_frame_id.c_str());

    sensor_msgs::PointCloud2 sens_msg_input_cloud_tr;
    //Transformation
    tf2::doTransform(input_cloud, sens_msg_input_cloud_tr, transformStamped);


    Cloud input_cloud_tr;
    //Conversion from sensor_msgs::PointCloud2 to Cloud
    pcl::fromROSMsg(sens_msg_input_cloud_tr,input_cloud_tr);


    //Delete the GroundPoints
    Indices object_indices;
    for( size_t i = 0; i < input_cloud_tr.size(); i++ ) {
        float z = input_cloud_tr.points[i].z;
        if( z > GND_LEVEL) {
            object_indices.push_back(i);
        }
    }
    Cloud output(input_cloud_tr, object_indices );
    return output;
}

std::vector<Cloud> findOrientation(Cloud fst_leg, Cloud snd_leg) {
    std::vector<Cloud> legs;
    Point fst_centroid, snd_centroid;

    pcl::computeCentroid (fst_leg, fst_centroid);
    pcl::computeCentroid (snd_leg, snd_centroid);

    if (fst_centroid.y > snd_centroid.y) {
        legs.push_back(fst_leg);
        legs.push_back(snd_leg);
    }
    else {
        legs.push_back(snd_leg);
        legs.push_back(fst_leg);
    }
    return legs;
}
std::vector<Cloud> splitCluster(Cloud both_legs) {
    std::vector<Cloud> legs;
    Point center;
    pcl::computeCentroid (both_legs, center);
    Indices left_inds, right_inds;
    for( size_t i = 0; i < both_legs.size(); i++ ) {
        float y = both_legs.points[i].y;
        if( y > center.y) {
            left_inds.push_back(i);
        }
        else {
            right_inds.push_back(i);
        }
    }

    Cloud left_leg(both_legs, left_inds);
    Cloud right_leg(both_legs, right_inds);
    legs.push_back(left_leg);
    legs.push_back(right_leg);
    return legs;
}


std::vector<Cloud> splitLegs(Cloud input_cloud) {

    //Cloud go wrong. Stated in here "http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9db5c9a9d98d6256c510d436693ab5f"
    Cloud_ptr input_cloud_ptr = input_cloud.makeShared();

    ROS_INFO("PointCloud before filtering has: %lu data points", input_cloud.points.size());
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<Point> vg;
    Cloud_ptr cloud_filtered(new Cloud);
    vg.setInputCloud (input_cloud_ptr);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filtered);
    ROS_INFO("PointCloud after filtering has: %lu data points", cloud_filtered->points.size());

    std::vector<Cloud> legs;


    if (cloud_filtered->empty()) {
        ROS_INFO("Input is empty");
        return legs;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud (cloud_filtered);

    //Setting the parameters for cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance (0.03); //3cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (1500);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);


    int i = 0;
    std::vector<Cloud> clusters;
    //Creating PointClouds for each cluster. clusters is sorted by the size of the cluster.
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        ROS_INFO("Cluster %d has %lu points", i++, cluster_indices.at(i).indices.size());
        Cloud cloud_cluster(*cloud_filtered, it->indices );
        clusters.push_back(cloud_cluster);
    }

    if (clusters.size() == 0) {
        ROS_INFO("NO CLUSTER FOUND");
    }
    else if (clusters.size() == 1) {
        ROS_INFO("ONE CLUSTER");
        legs = splitCluster(clusters[0]);
    }
    else if (clusters.size() == 2) {
        Cloud fst_leg = clusters[0];
        Cloud snd_leg = clusters[1];
        legs = findOrientation(fst_leg, snd_leg);
    }
    else {
        ROS_INFO("More than 2 Clusters detected. Only the 2 largest will be published.");
        Cloud fst_leg = clusters[0];
        Cloud snd_leg = clusters[1];
        legs = findOrientation(fst_leg, snd_leg);
    }
    return legs;
}


void cloud_cb (sensor_msgs::PointCloud2 input_cloud)
{
    Cloud removedGround = removeGround(input_cloud);
    std::vector<Cloud> legs = splitLegs(removedGround);
    if (!legs.empty()) {
        Cloud left_leg = legs[0];
        Cloud right_leg = legs[1];
//     PointStamped left_toe = findToe(left_leg);
//     PointStamped right_toe = findToe(right_leg);
        pub_left_leg.publish(left_leg);
        pub_right_leg.publish(right_leg);
    }
}


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "feet_detection");
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

    pub_left_leg = nh.advertise<sensor_msgs::PointCloud2> ("left_leg", 1);
    pub_right_leg = nh.advertise<sensor_msgs::PointCloud2> ("right_leg", 1);


    // Spin
    ros::spin();
}
