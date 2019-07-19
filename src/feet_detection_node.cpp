#include "pcl_types.h"

int Icp_error_count = 0, icp_count = 0, init_frontal_frame = 0, init_side_frame = 0;

geometry_msgs::TransformStamped transformStamped;
ros::Publisher pub_left_leg, pub_right_leg, pub_left_toe, pub_right_toe, pub_right_sole, pub_left_sole, pub_LeftFoot, pub_RightFoot, pub_left_heel, pub_right_heel, pub_left_ankle, pub_right_ankle, pub_foot_strip;
geometry_msgs::PointStamped oldLeft,oldRight;
Cloud_ptr right_foot_reference, left_foot_reference;
Cloud side_init_cloud;
bool init_front_done = false, init_side_done = false;

float GND_LEVEL, ICP_FITNESS_THRESHHOLD, CLUSTER_TOLERANCE, POINT_SIZE, FOOT_HEIGHT, footLength;
int MIN_CLUSTER_SIZE, INIT_FRONTAL_CAPTURE_FRAME, INIT_SIDE_CAPTURE_FRAME,  INIT_SIDE_END_FRAME;

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

//      ROS_INFO("PointCloud before filtering has: %lu data points", input_cloud.points.size());
    // Create the filtering object: downsample the dataset using a leaf size of 1cm
    pcl::VoxelGrid<Point> vg;
    Cloud_ptr cloud_filtered(new Cloud);
    vg.setInputCloud (input_cloud_ptr);
    vg.setLeafSize (POINT_SIZE, POINT_SIZE, POINT_SIZE);
    vg.filter (*cloud_filtered);
//      ROS_INFO("PointCloud after filtering has: %lu data points", ilcloud_filtered->points.size());

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

float getFootHeight(Cloud leg, Point toe) {
    float height= 0;

    Indices indices;
    for (int i = 0; i < leg.size(); i++) {
        if (std::abs(leg.points[i].x - toe.x) < 0.03) indices.push_back(i);
    }

    Cloud footStrip(leg,indices);

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    ne.setInputCloud (footStrip.makeShared());

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.03);

    // Compute the features
    ne.compute (*cloud_normals);

    

    pub_foot_strip.publish(footStrip);

    return height;
}

void front_init(Cloud left_leg,Cloud right_leg) {
    init_frontal_frame++;
    if (init_frontal_frame < INIT_FRONTAL_CAPTURE_FRAME) {
        ROS_INFO("FRONTAL_INIT IN %i FRAMES", INIT_FRONTAL_CAPTURE_FRAME - init_frontal_frame);
        return;
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
        init_front_done = true;
        ROS_INFO("FRONTAL INIT SUCCESSFUL");
    }
    return;
}

//The side init only uses the left leg. So for the init you have to turn right (So the left leg is in front) and place your right leg a bit away from the left leg (It has to be more right than the left leg).
void side_init(Cloud cloud) {
    init_side_frame++;
    if (init_side_frame < INIT_SIDE_CAPTURE_FRAME) {
        ros::param::get("point_size_pre_init", POINT_SIZE);
        ros::param::get("min_cluster_size_pre_init", MIN_CLUSTER_SIZE);
        ROS_INFO("RECORDING BEGINS IN %i FRAMES", INIT_SIDE_CAPTURE_FRAME - init_side_frame);
    }
    else if (init_side_frame < INIT_SIDE_END_FRAME) {
        ROS_INFO("RECORDING ENDS IN %i FRAMES", INIT_SIDE_END_FRAME - init_side_frame);
        side_init_cloud += cloud;
    }
    else if (init_side_frame == INIT_SIDE_END_FRAME) {
        ROS_INFO("Processing the Side Init");
        ROS_INFO("PointCloud before filtering has: %lu data points", side_init_cloud.points.size());
        ros::param::get("point_size_pre_init", POINT_SIZE);
        ros::param::get("min_cluster_size_pre_init", MIN_CLUSTER_SIZE);
        pcl::VoxelGrid<Point> vg;
        Cloud_ptr cloud_filtered(new Cloud);
        vg.setInputCloud (side_init_cloud.makeShared());
        vg.setLeafSize (POINT_SIZE, POINT_SIZE, POINT_SIZE);
        vg.filter (*cloud_filtered);
     ROS_INFO("PointCloud after filtering has: %lu data points", cloud_filtered->points.size());

        std::vector<Cloud> legs;


        if (cloud_filtered->empty()) {
            ROS_INFO("Side Init Input is empty");
        }

        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<Point>::Ptr tree (new pcl::search::KdTree<Point>);
        tree->setInputCloud (cloud_filtered);

        //Setting the parameters for cluster extraction
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<Point> ec;
        ec.setClusterTolerance(CLUSTER_TOLERANCE);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_filtered);
        ec.extract(cluster_indices);

        std::vector<Cloud> clusters;
        //Creating PointClouds for each cluster. clusters is sorted by the size of the cluster.
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
            Cloud cloud_cluster(*cloud_filtered, it->indices );
            clusters.push_back(cloud_cluster);
        }

        std::vector<Cloud> both_legs = findOrientation(clusters[0], clusters[1]);
        Cloud left_leg = both_legs[0];
        left_leg.header.frame_id = "base_link";
        Cloud right_leg = both_legs[1];
        right_leg.header.frame_id = "base_link";
        pub_left_leg.publish(left_leg);
        pub_right_leg.publish(right_leg);
        
        Cloud leg = both_legs[1];

        Cloud legCropped = leg;
        //Calculating the length of the foot. Could be usefull later
        Point maxY = legCropped.points[0], minY = legCropped.points[0];
        for (int i = 0; i < legCropped.size(); i ++) {
            if (legCropped.points[i].y > maxY.y) maxY = legCropped.points[i];
            if (legCropped.points[i].y < minY.y) minY = legCropped.points[i];
        }
        ROS_INFO("CLUSTERING DONE GETTING FOOT HEIGHT");
        float footHeight = getFootHeight(leg, maxY);
        ROS_INFO("SIDE INIT SUCCESSFUL");
        ROS_INFO("The foot is %fcm long", maxY.y - minY.y);
        ros::param::get("point_size_post_init", POINT_SIZE);
        ros::param::get("min_cluster_size_post_init", MIN_CLUSTER_SIZE);
        footLength = maxY.y - minY.y;
        init_side_done = true;
    }
}

void removeRow(Eigen::MatrixXf& matrix, unsigned int rowToRemove) {
    unsigned int numRows = matrix.rows()-1;
    unsigned int numCols = matrix.cols();

    if( rowToRemove < numRows )
        matrix.block(rowToRemove,0,numRows-rowToRemove,numCols) = matrix.block(rowToRemove+1,0,numRows-rowToRemove,numCols);

    matrix.conservativeResize(numRows,numCols);
}

void removeColumn(Eigen::MatrixXf& matrix, unsigned int colToRemove) {
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
        matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
}

geometry_msgs::PointStamped findHeel(geometry_msgs::PointStamped toe, Eigen::MatrixXf footPose) {
    geometry_msgs::PointStamped heel;
    heel.header.stamp = ros::Time::now();
    heel.header.frame_id = "base_link";
    heel.header.seq++;

    Eigen::Vector3f footVector(-footLength,0.0,0.0);
//     std::cout << "Transformation was :\n" << footPose << std::endl;
    removeColumn(footPose, 3);
    removeRow(footPose, 3);
//     std::cout << "Rotation is :\n" << footPose << std::endl;
    Eigen::Vector3f heelDirection = footPose * footVector;
    heel.point.x = toe.point.x + heelDirection(0);
    heel.point.y = toe.point.y + heelDirection(1);
    heel.point.z = toe.point.z + heelDirection(2);

    return heel;
}

geometry_msgs::PointStamped findAnkle(geometry_msgs::PointStamped heel) {
    geometry_msgs::PointStamped ankle;
    ankle.header.stamp = ros::Time::now();
    ankle.header.frame_id = "base_link";
    ankle.header.seq++;

    ankle.point.x = heel.point.x + footLength/3;
    ankle.point.y = heel.point.y;
    ankle.point.z = heel.point.z + FOOT_HEIGHT;

    return ankle;
}

bool init() {
    return init_side_done && init_front_done;
}

void cloud_cb (sensor_msgs::PointCloud2 input_cloud) {

    Cloud removedGround = removeGround(input_cloud);
    if (!removedGround.empty() && !init_side_done) side_init(removedGround);
    std::vector<Cloud> legs;
    if (init_side_done) legs = splitLegs(removedGround);
    if (init_side_done && !init_front_done && !legs.empty()) front_init(legs[0], legs[1]);
    if (init() && !legs.empty()) {
        Cloud left_leg = legs[0];
        Cloud right_leg = legs[1];
        if (!left_leg.empty()) {
            pub_left_leg.publish(left_leg);
            geometry_msgs:: PointStamped left_toe = findToe(left_leg, true);
            pub_left_toe.publish(left_toe);
            Eigen::Matrix4f leftFootTrans = findFootTransformation(left_leg, true);
            geometry_msgs::PointStamped left_heel = findHeel(left_toe, leftFootTrans);
            pub_left_heel.publish(left_heel);
            geometry_msgs::PointStamped left_ankle = findAnkle(left_heel);
            pub_left_ankle.publish(left_ankle);
        }
        if (init() && !right_leg.empty()) {
            pub_right_leg.publish(right_leg);
            geometry_msgs:: PointStamped right_toe = findToe(right_leg, false);
            pub_right_toe.publish(right_toe);
            Eigen::Matrix4f rightFootTrans = findFootTransformation(right_leg, false);
            geometry_msgs::PointStamped right_heel = findHeel(right_toe, rightFootTrans);
            pub_right_heel.publish(right_heel);
            geometry_msgs::PointStamped right_ankle = findAnkle(right_heel);
            pub_right_ankle.publish(right_ankle);
        }
    }
//     float success_percent = ((float) Icp_error_count /(float) (icp_count));
//     ROS_INFO("ICP's error rate is: %f",success_percent);
}


bool init_reset(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response) {
    ROS_INFO("RESET INIT");
    init_front_done = false;
    init_side_done = false;
    init_side_frame = 0;
    init_frontal_frame = 0;
    response.success = true;
    response.message = "The initialisation will restart now!";
    return true;
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

    pub_left_heel = nh.advertise<geometry_msgs::PointStamped>("left_heel", 1);
    pub_right_heel = nh.advertise<geometry_msgs::PointStamped>("right_heel", 1);

    pub_left_ankle = nh.advertise<geometry_msgs::PointStamped>("left_ankle", 1);
    pub_right_ankle = nh.advertise<geometry_msgs::PointStamped>("right_ankle", 1);

    ros::ServiceServer service = nh.advertiseService("camera_lower_leg_tracking/init_reset", init_reset);

    pub_foot_strip = nh.advertise<sensor_msgs::PointCloud2>("footStrip", 1);



    ros::param::get("ground_level", GND_LEVEL);
    ros::param::get("min_cluster_size_pre_init", MIN_CLUSTER_SIZE);
    ros::param::get("cluster_tolerance", CLUSTER_TOLERANCE);
    ros::param::get("point_size_pre_init", POINT_SIZE);
    ros::param::get("icp_fitness_threshhold", ICP_FITNESS_THRESHHOLD);
    ros::param::get("init_frontal_capture_frame", INIT_FRONTAL_CAPTURE_FRAME);
    ros::param::get("init_side_capture_frame", INIT_SIDE_CAPTURE_FRAME);
    ros::param::get("init_side_end_frame", INIT_SIDE_END_FRAME);
    ros::param::get("foot_height", FOOT_HEIGHT);


    // Spin
    ros::spin();
    return 0;
}
