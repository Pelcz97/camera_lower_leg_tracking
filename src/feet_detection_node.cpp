#include "ros/ros.h"
#include "../include/pcl_types.h"

#define GND_LEVEL (0.02)
#define MIN_X_CAMERA (-0.8)
#define POINT_SIZE (0.01)
#define CLUSTER_TOLERANCE (0.03)
#define MIN_CLUSTER_SIZE (100)
#define MAX_CLUSTER_SIZE (1000)

geometry_msgs::TransformStamped transformStamped;
ros::Publisher pub_left_leg, pub_right_leg, pub_left_toe, pub_right_toe, pub_right_sole, pub_left_sole;
geometry_msgs::PointStamped oldLeft,oldRight;

Cloud removeGround(sensor_msgs::PointCloud2 input_cloud) {
//     ROS_INFO("Tranform frame is %s und %s", transformStamped.header.frame_id.c_str(), transformStamped.child_frame_id.c_str());

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

//side can only be left or right!
 geometry_msgs:: PointStamped findToe(Cloud input_cloud, int left) {
    geometry_msgs::PointStamped oldPoint;
    if (left) {
        oldPoint = oldLeft;
    }
    else {
        oldPoint = oldRight;
    }
     
    geometry_msgs::PointStamped maxX;
    maxX.header.stamp = ros::Time::now();
    maxX.header.frame_id = "base_link";
    maxX.header.seq++;
    maxX.point.x = input_cloud.points[0].x;
    maxX.point.y = input_cloud.points[0].y;
    maxX.point.z = input_cloud.points[0].z;

    for( size_t i = 0; i < input_cloud.size(); i++ ){
    	float X = input_cloud.points[i].x;
        if( X > maxX.point.x) {
            maxX.point.x = input_cloud.points[i].x;
            maxX.point.y = input_cloud.points[i].y;
            maxX.point.z = input_cloud.points[i].z;
        }
    }
    
    float distance = sqrt(pow(maxX.point.x - oldPoint.point.x, 2) + pow(maxX.point.y - oldPoint.point.y, 2) + pow(maxX.point.z - maxX.point.z, 2));
    if (distance > 0.1){
//         ROS_INFO("The distance is: %6.4lf and it's too big!", distance);
    }

    if (left) {
        oldLeft = maxX;
    }
    else {
        oldRight = maxX;
    }
    
    return maxX;
}

Cloud findlowestPoints(Cloud input_cloud) {
    
    Point lowest;
    Cloud lowestPoints;
    
    if (!input_cloud.empty()) {
        lowest.x = input_cloud.points[0].x;
        lowest.y = input_cloud.points[0].y;
        lowest.z = input_cloud.points[0].z;
        for( size_t i = 0; i < input_cloud.size(); i++ ){
            float Z = input_cloud.points[i].z;
            if( Z < lowest.z) {
                lowest.x = input_cloud.points[i].x;
                lowest.y = input_cloud.points[i].y;
                lowest.z = input_cloud.points[i].z;
            }
        }
        for( size_t i = 0; i < input_cloud.size(); i++ ){
            float Z = input_cloud.points[i].z;
            if(Z < lowest.z +  POINT_SIZE) {
                lowestPoints.push_back(input_cloud.points[i]);
            }
        }
    }
    return lowestPoints;
}

//side can only be left or right!
Cloud findSole(Cloud input, int left) {
    Cloud result,soleWithOutlier; 
        geometry_msgs::Point toe = findToe(input, left).point;
        double maxX = toe.x;
        double minX = maxX -  POINT_SIZE;
        double midY = toe.y;
        do {
            Cloud sliceLeft, sliceRight;
            for (int i = 0; i < input.points.size(); i++) {
                if (input.points[i].x >= minX && input.points[i].x < maxX) {
                    if (input.points[i].y >= midY) sliceLeft.push_back(input.points[i]);
                    else sliceLeft.push_back(input.points[i]);
                }
            }
            soleWithOutlier += findlowestPoints(sliceLeft);
            soleWithOutlier += findlowestPoints(sliceRight);
            maxX = minX;
            minX = maxX -  POINT_SIZE;
        } while (minX >= MIN_X_CAMERA);
                
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud(soleWithOutlier.makeShared());

    //Setting the parameters for cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance (0.02);
    ec.setSearchMethod (tree);
    ec.setInputCloud (soleWithOutlier.makeShared());
    ec.extract (cluster_indices);

    if(!cluster_indices.empty()) {
        Cloud temp(soleWithOutlier, cluster_indices[0].indices);
        result = temp;   
    }
        
    result.header.frame_id = "base_link";
    return result;
}

pcl::ModelCoefficients findSolePlane(Cloud sole) {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    if (!sole.points.empty()) {
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        // Create the segmentation object
        pcl::SACSegmentation<Point> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);

        seg.setInputCloud (sole.makeShared());
        seg.segment (*inliers, *coefficients);
    }
    return *coefficients;

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
//     ROS_INFO("CLUSTER HAS %lu POINTS", both_legs.points.size());
    std::vector<Cloud> legs;
    if (both_legs.points.size() > 1000) {
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
    }
    return legs;
}


std::vector<Cloud> splitLegs(Cloud input_cloud) {

    //Cloud go wrong. Stated in here "http://docs.pointclouds.org/trunk/classpcl_1_1_point_cloud.html#ab9db5c9a9d98d6256c510d436693ab5f"
    Cloud_ptr input_cloud_ptr = input_cloud.makeShared();

//     ROS_INFO("PointCloud before filtering has: %lu data points", input_cloud.points.size());
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<Point> vg;
    Cloud_ptr cloud_filtered(new Cloud);
    vg.setInputCloud (input_cloud_ptr);
    vg.setLeafSize (POINT_SIZE, POINT_SIZE, POINT_SIZE);
    vg.filter (*cloud_filtered);
//     ROS_INFO("PointCloud after filtering has: %lu data points", cloud_filtered->points.size());

    std::vector<Cloud> legs;


    if (cloud_filtered->empty()) {
//         ROS_INFO("Input is empty");
        return legs;
    }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
    tree->setInputCloud (cloud_filtered);

    //Setting the parameters for cluster extraction
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<Point> ec;
    ec.setClusterTolerance(CLUSTER_TOLERANCE);
    ec.setMaxClusterSize(MAX_CLUSTER_SIZE);
    ec.setMinClusterSize(MIN_CLUSTER_SIZE);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_filtered);
    ec.extract(cluster_indices);


//     int i = 0;
    std::vector<Cloud> clusters;
    //Creating PointClouds for each cluster. clusters is sorted by the size of the cluster.
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
//         ROS_INFO("Cluster %d has %lu points", i++, cluster_indices.at(i).indices.size());
        Cloud cloud_cluster(*cloud_filtered, it->indices );
        clusters.push_back(cloud_cluster);
    }

    if (clusters.size() == 0) {
//         ROS_INFO("NO CLUSTER FOUND");
    }
    else if (clusters.size() == 1) {
//         ROS_INFO("ONE CLUSTER");
        legs = splitCluster(clusters[0]);
    }
    else if (clusters.size() == 2) {
        Cloud fst_leg = clusters[0];
        Cloud snd_leg = clusters[1];
        legs = findOrientation(fst_leg, snd_leg);
    }
    else {
//         ROS_INFO("More than 2 Clusters detected. Only the 2 largest will be published.");
        Cloud fst_leg = clusters[0];
        Cloud snd_leg = clusters[1];
        legs = findOrientation(fst_leg, snd_leg);
    }
    return legs;
}


void cloud_cb (sensor_msgs::PointCloud2 input_cloud) {
    Cloud removedGround = removeGround(input_cloud);
    std::vector<Cloud> legs = splitLegs(removedGround);
    if (!legs.empty()) {
        Cloud left_leg = legs[0];
        Cloud right_leg = legs[1];                
        pub_left_leg.publish(left_leg);
        pub_right_leg.publish(right_leg);

        
        geometry_msgs:: PointStamped left_toe = findToe(left_leg, true);
        geometry_msgs:: PointStamped right_toe = findToe(right_leg, false);
        pub_left_toe.publish(left_toe);
        pub_right_toe.publish(right_toe);
        
        Cloud leftSole = findSole(left_leg, true);
        Cloud rightSole = findSole(right_leg, false);
        pub_left_sole.publish(leftSole);
        pub_right_sole.publish(rightSole);
        
        pcl::ModelCoefficients leftSolePlane = findSolePlane(leftSole);
        pcl::ModelCoefficients rightSolePlane = findSolePlane(rightSole);
        
        if (!leftSolePlane.values.empty()) {
            ROS_INFO("ModelCoefficients for the LeftSole are: %f %f %f %f", leftSolePlane.values[1],leftSolePlane.values[2],leftSolePlane.values[3],leftSolePlane.values[4]);
        }
        
        if (!rightSolePlane.values.empty()) {
            ROS_INFO("ModelCoefficients for the RightSole are: %f %f %f %f", rightSolePlane.values[1],rightSolePlane.values[2],rightSolePlane.values[3],rightSolePlane.values[4]);
        }
    }
}

int main (int argc, char** argv) {
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

    pub_left_leg = nh.advertise<sensor_msgs::PointCloud2>("left_leg", 1);
    pub_right_leg = nh.advertise<sensor_msgs::PointCloud2>("right_leg", 1);
    
    pub_left_toe = nh.advertise<geometry_msgs::PointStamped>("left_toe", 1);
    pub_right_toe = nh.advertise<geometry_msgs::PointStamped>("right_toe", 1);
    
    pub_left_sole = nh.advertise<sensor_msgs::PointCloud2>("left_sole", 1);
    pub_right_sole = nh.advertise<sensor_msgs::PointCloud2>("right_sole", 1);
    
    // Spin
    ros::spin();
    return 0;
}
