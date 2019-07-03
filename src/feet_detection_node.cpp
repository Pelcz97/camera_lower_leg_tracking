#include "pcl_types.h"

int Icp_error_count = 0, icp_count = 0, init_frontal_frame = 0, init_side_frame = 0;

geometry_msgs::TransformStamped transformStamped;
ros::Publisher pub_left_leg, pub_right_leg, pub_left_toe, pub_right_toe, pub_right_sole, pub_left_sole, pub_LeftFoot, pub_RightFoot;
geometry_msgs::PointStamped oldLeft,oldRight;
bool init_done = false;
Cloud_ptr right_foot_reference, left_foot_reference;

float GND_LEVEL, ICP_FITNESS_THRESHHOLD, CLUSTER_TOLERANCE, POINT_SIZE, FOOT_HEIGHT;
int MIN_CLUSTER_SIZE, INIT_FRONTAL_CAPTURE_FRAME, INIT_SIDE_CAPTURE_FRAME;

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

    geometry_msgs::PointStamped maxX;
    maxX.header.stamp = ros::Time::now();
    maxX.header.frame_id = "base_link";
    maxX.header.seq++;
    if (!input_cloud.empty()) {
        maxX.point.x = input_cloud.points[0].x;
        maxX.point.y = input_cloud.points[0].y;
        maxX.point.z = input_cloud.points[0].z;

        for( size_t i = 0; i < input_cloud.size(); i++ ) {
            float X = input_cloud.points[i].x;
            if( X > maxX.point.x) {
                maxX.point.x = input_cloud.points[i].x;
                maxX.point.y = input_cloud.points[i].y;
                maxX.point.z = input_cloud.points[i].z;
            }
        }
    }
    geometry_msgs::PointStamped oldPoint;
    if (left) {
        oldPoint = oldLeft;
    }
    else {
        oldPoint = oldRight;
    }

    float distance = sqrt(pow(maxX.point.x - oldPoint.point.x, 2) + pow(maxX.point.y - oldPoint.point.y, 2) + pow(maxX.point.z - maxX.point.z, 2));
    if (distance > 0.1) {
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

Eigen::Matrix4f findFootTransformation(Cloud input, int left) {

    icp_count++;
    pcl::IterativeClosestPoint<Point, Point> icp;
    if (left) icp.setInputSource(left_foot_reference);
    else icp.setInputSource(right_foot_reference);
    icp.setInputTarget(input.makeShared());
    icp.setMaximumIterations (100);
    Cloud Final;
    icp.align(Final);
//     ROS_INFO("ICP has converged: %i with the score: %f", icp.hasConverged(), icp.getFitnessScore());
    if (icp.getFitnessScore() <= ICP_FITNESS_THRESHHOLD) {
        Eigen::Matrix4f transformation = icp.getFinalTransformation ();
        Final.header.frame_id = "base_link";

        if (left) pub_LeftFoot.publish(Final);
        else pub_RightFoot.publish(Final);

        return transformation;
    }
    else Icp_error_count++;
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
    Point center;
    pcl::computeCentroid (both_legs, center);
    Cloud left_leg, right_leg;
    if (both_legs.points.size() > 1000) {
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

        left_leg = Cloud(both_legs, left_inds);
        right_leg= Cloud(both_legs, right_inds);
        legs.push_back(left_leg);
        legs.push_back(right_leg);
        return legs;
    }
    if (center.y >= 0) {
        ROS_INFO("ONLY LEFT LEG IN FRAME");
        legs.push_back(both_legs);
        legs.push_back(right_leg);
    }
    else {
        ROS_INFO("ONLY RIGHT LEG IN FRAME");
        legs.push_back(left_leg);
        legs.push_back(both_legs);
    }
    return legs;
}

//With splitLegs a vector will be filled where the first entry is left and and the second is right.
//If there is no Cluster then legs will stay empty.
//If both legs are one cluster it will get split in the middle
//If only one leg is in the frame its Cloud will be as expected. The other Cloud will be empty.
std::vector<Cloud> splitLegs(Cloud input_cloud) {

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
    ec.setMinClusterSize (MIN_CLUSTER_SIZE);
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
//         ROS_INFO("ONE CLUSTER with Size: %lu", clusters[0].size());
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


bool init_front(Cloud left_leg,Cloud right_leg) {
    init_frontal_frame++;
    if (init_frontal_frame < INIT_FRONTAL_CAPTURE_FRAME) {
        ROS_INFO("FRONTAL_INIT IN %i FRAMES", INIT_FRONTAL_CAPTURE_FRAME - init_frontal_frame);
        return false;
    }
    if (init_frontal_frame == INIT_FRONTAL_CAPTURE_FRAME) {
        Indices left_leg_inds, right_leg_inds;
        for (int i = 0; i < left_leg.size(); i++) {
            if (left_leg.points[i].z <= FOOT_HEIGHT) left_leg_inds.push_back(i);
        }
        Cloud leftlegCropped(left_leg, left_leg_inds);
        for (int i = 0; i < right_leg.size(); i++) {
            if (right_leg.points[i].z <= FOOT_HEIGHT) right_leg_inds.push_back(i);
        }
        Cloud rightLegCropped(right_leg, right_leg_inds);
        left_foot_reference = leftlegCropped.makeShared();
        right_foot_reference = rightLegCropped.makeShared();
        ROS_INFO("FRONTAL INIT SUCCESSFUL");
    }
    return true;
}

//The side init only uses the left leg. So for the init you have to turn right (So the left leg is in front) and place your right leg a bit away from the left leg (It has to be more right than the left leg).
bool init_side(Cloud left_leg) {
    init_side_frame++;
    if (init_side_frame < INIT_SIDE_CAPTURE_FRAME) {
        ROS_INFO("SIDE_INIT IN %i FRAMES", INIT_SIDE_CAPTURE_FRAME - init_side_frame);
        return false;
    }
    if (init_side_frame == INIT_SIDE_CAPTURE_FRAME) {
        Indices left_leg_inds;
        for (int i = 0; i < left_leg.size(); i++) {
            if (left_leg.points[i].z <= FOOT_HEIGHT) left_leg_inds.push_back(i);
        }
        Cloud leftlegCropped(left_leg, left_leg_inds);
        //Calculating the length of the foot. Could be usefull later
        float maxY = leftlegCropped.points[0].y, minY = leftlegCropped.points[0].y;
        for (int i = 0; i < leftlegCropped.size(); i ++) {
            if (leftlegCropped.points[i].y > maxY) maxY = leftlegCropped.points[i].y;
            if (leftlegCropped.points[i].y < minY) minY = leftlegCropped.points[i].y;
        }
        ROS_INFO("SIDE INIT SUCCESSFUL");
        ROS_INFO("The foot is %fcm long", maxY - minY);
    }
    return true;
}

bool init(Cloud left_leg, Cloud right_leg) {
    return init_front(left_leg, right_leg) && init_side(left_leg);
}

void cloud_cb (sensor_msgs::PointCloud2 input_cloud) {

    Cloud removedGround = removeGround(input_cloud);
    std::vector<Cloud> legs = splitLegs(removedGround);
    if (!legs.empty()) {
        Cloud left_leg = legs[0];
        Cloud right_leg = legs[1];
        if (!init_done && !left_leg.empty() && !right_leg.empty()) {
            init_done = init(left_leg, right_leg);
            pub_left_leg.publish(left_leg);
            pub_right_leg.publish(right_leg);
        }
        if (init_done && !left_leg.empty()) {
            pub_left_leg.publish(left_leg);
            geometry_msgs:: PointStamped left_toe = findToe(left_leg, true);
            pub_left_toe.publish(left_toe);
            Eigen::Matrix4f leftFootTrans = findFootTransformation(left_leg, true);
        }
        if (init_done && !right_leg.empty()) {
            pub_right_leg.publish(right_leg);
            geometry_msgs:: PointStamped right_toe = findToe(right_leg, false);
            pub_right_toe.publish(right_toe);
            Eigen::Matrix4f rightFootTrans = findFootTransformation(right_leg, false);
        }
    }
//     float success_percent = ((float) Icp_error_count /(float) (icp_count));
//     ROS_INFO("ICP's error rate is: %f",success_percent);
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

    pub_LeftFoot = nh.advertise<sensor_msgs::PointCloud2>("left_Foot",1);
    pub_RightFoot = nh.advertise<sensor_msgs::PointCloud2>("right_Foot",1);

    ros::param::get("ground_level", GND_LEVEL);
    ros::param::get("min_cluster_size", MIN_CLUSTER_SIZE);
    ros::param::get("cluster_tolerance", CLUSTER_TOLERANCE);
    ros::param::get("point_size", POINT_SIZE);
    ros::param::get("icp_fitness_threshhold", ICP_FITNESS_THRESHHOLD);
    ros::param::get("init_frontal_capture_frame", INIT_FRONTAL_CAPTURE_FRAME);
    ros::param::get("init_side_capture_frame", INIT_SIDE_CAPTURE_FRAME);
    ros::param::get("foot_height", FOOT_HEIGHT);


    // Spin
    ros::spin();
    return 0;
}
