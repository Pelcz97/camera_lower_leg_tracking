
#include "ros/ros.h"
#include "../include/pcl_types.h"
#include <Eigen/Dense>
#include <mutex>


#define GND_LEVEL (0.01)
#define POINT_SIZE (0.01)
#define CLUSTER_TOLERANCE (0.03)
#define SHIN_HEIGHT (0.178)
#define SHIN_TOLERANCE (0.06)

KalmanFilter* kFilter_left;
KalmanFilter* kFilter_right;
KalmanFilter* kFilter_LSR_left;
KalmanFilter* kFilter_LSR_right;

ros::Time previous_T_left, previous_T_right;
ros::Time previous_T_left_seq,previous_T_right_seq;
double shinToeDist = 0;

std::vector< std::vector<double>  > x_hats(2, std::vector<double> (3,0));
bool right_init = false,left_init = false;
bool sequentiell = false;
Eigen::MatrixXd R_loc;

std::mutex mtx_left,mtx_right;

geometry_msgs::TransformStamped transformStamped;
ros::Publisher pub_left_leg, pub_right_leg, pub_left_toe, pub_right_toe, pub_left_toe_kf, pub_right_toe_kf, pub_left_toe_seq, pub_right_toe_seq;
// geometry_msgs::PointStamped old_right_toe, old_left_toe;
// float addedDistancesLeft = 0, addedDistancesRight = 0;
// int iterations = 0;

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


//side can only be left or right!
geometry_msgs:: PointStamped findShin(Cloud input_cloud) {

    std::vector<int> cloud_inds;
    geometry_msgs::PointStamped maxX;
    maxX.header.stamp = ros::Time::now();
    maxX.header.frame_id = "base_link";
    maxX.header.seq++;
    double height_diff;
    if (!input_cloud.empty()) {

        for(int i = 0; i < input_cloud.size(); i++ ) {
            height_diff = std::abs(input_cloud.points[i].z - SHIN_HEIGHT);
            if (height_diff > SHIN_TOLERANCE) continue;
            else {
                if (cloud_inds.empty()) {
                    maxX.point.x = input_cloud.points[i].x;
                    maxX.point.y = input_cloud.points[i].y;
                    maxX.point.z = input_cloud.points[i].z;
                }
                else {
                    float X = input_cloud.points[i].x;
                    if( X > maxX.point.x) {
                        maxX.point.x = input_cloud.points[i].x;
                        maxX.point.y = input_cloud.points[i].y;
                        maxX.point.z = input_cloud.points[i].z;
                    }
                }
                cloud_inds.push_back(i);
            }
        }
        Indices inds;
        for (int i = 0; i < cloud_inds.size(); i++) {
            int cloud_ind = cloud_inds[i];
            if  ((input_cloud.points[cloud_ind].x + 0.02) >= maxX.point.x) {
                inds.push_back(i);
            }
        }
        ROS_INFO("Found %d shin candidates", cloud_inds.size());
        Cloud frontalPoints(input_cloud, inds);
        Point centroid;
        pcl::computeCentroid(frontalPoints, centroid);
        maxX.point.y = centroid.y;
        maxX.point.z = centroid.z;
    }
    return maxX;
}

geometry_msgs:: PointStamped findToe(Cloud input_cloud) {

    geometry_msgs::PointStamped maxX;
    maxX.header.stamp = ros::Time::now();
    maxX.header.frame_id = "base_link";
    maxX.header.seq++;
    if (!input_cloud.empty()) {
        maxX.point.x = input_cloud.points[0].x;
        maxX.point.y = input_cloud.points[0].y;
        maxX.point.z = input_cloud.points[0].z;

        for(size_t i = 0; i < input_cloud.size(); i++ ) {
            float X = input_cloud.points[i].x;
            if( X > maxX.point.x) {
                maxX.point.x = input_cloud.points[i].x;
                maxX.point.y = input_cloud.points[i].y;
                maxX.point.z = input_cloud.points[i].z;
            }
        }
        Indices inds;
        for (int i = 0; i < input_cloud.size(); i++) {
            if  ((input_cloud.points[i].x + 0.02) >= maxX.point.x) {
                inds.push_back(i);
            }
        }
        Cloud frontalPoints(input_cloud, inds);
        Point centroid;
        pcl::computeCentroid(frontalPoints, centroid);
        maxX.point.y = centroid.y;
    }
    return maxX;
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
        ROS_INFO("Input is empty");
        return legs;
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
    ec.setMinClusterSize(200);
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

geometry_msgs::PointStamped vecToPointStamped(std::vector<double> vec, std_msgs::Header header) {
    geometry_msgs::PointStamped msg;
    msg.header = header;
    
    msg.point.x = vec[0];
    msg.point.y = vec[1];
    msg.point.z = vec[2];
    
    return msg;
}

void cloud_cb (sensor_msgs::PointCloud2 input_cloud) {
    ros::Time start = ros::Time::now();
    Cloud removedGround = removeGround(input_cloud);
    std::vector<Cloud> legs = splitLegs(removedGround);
    ROS_INFO("TOE callback");
    if (legs.size() == 2) {
        Cloud left_leg = legs[0];
        Cloud right_leg = legs[1];
//         iterations++;
        if (!left_leg.empty()) {
            geometry_msgs:: PointStamped left_toe = findToe(left_leg);
            ROS_INFO("LEFT LEG");
            //geometry_msgs:: PointStamped left_shin = findShin(left_leg);
//             float distance = sqrt(pow(old_left_toe.point.x - left_toe.point.x, 2) + pow(old_left_toe.point.y - left_toe.point.y, 2) + pow(old_left_toe.point.z - left_toe.point.z, 2));
//             addedDistancesLeft += std::abs(distance);
//             old_left_toe = left_toe;
            if (!kFilter_left->isInitializated()) {
                ROS_INFO("INIT LEFT FILTER");
                std::vector<double> toeLeft;
                toeLeft.push_back(left_toe.point.x);
                toeLeft.push_back(left_toe.point.y);
                toeLeft.push_back(left_toe.point.z);
                toeLeft.push_back(0);
                toeLeft.push_back(0);
                toeLeft.push_back(0);
                previous_T_left = ros::Time::now();
                previous_T_left_seq = ros::Time::now();
                x_hats[0] = toeLeft;
                
                if (!kFilter_left->configure(toeLeft)) ROS_ERROR("KalmanFilter for the left Toe couldn't configure!");
                
            }
            else {
                ROS_INFO("LEFT TOE Measured");
                std::vector<double> data_in_l, data_out_l,data_out_l_seq;
                data_in_l.push_back(left_toe.point.x);
                data_in_l.push_back(left_toe.point.y);
                data_in_l.push_back(left_toe.point.z);
                ros::Time now_left = ros::Time::now();
                double delta_t_left = (now_left - previous_T_left).toSec();
                
                
                kFilter_left->update(data_in_l, data_out_l, delta_t_left, true, R_loc);
                previous_T_left = now_left;
                
                geometry_msgs::PointStamped msg = vecToPointStamped(data_out_l, left_toe.header);
                pub_left_toe_kf.publish(msg);

                mtx_left.lock();
                if(kFilter_LSR_left->isInitializated()) {
                    double delta_t_left_seq = (now_left - previous_T_left_seq).toSec();
                    kFilter_LSR_left->update(data_in_l, data_out_l_seq, delta_t_left_seq, true, R_loc);
                    x_hats[0] = data_out_l_seq;
                    previous_T_left_seq = now_left;

                    geometry_msgs::PointStamped msg_seq = vecToPointStamped(data_out_l_seq, left_toe.header);
                    
                    pub_left_toe_seq.publish(msg_seq);
                }
                left_init = true;
                mtx_left.unlock();
            }
            pub_left_leg.publish(left_leg);
            pub_left_toe.publish(left_toe);
        }
        if (!right_leg.empty()) {
            geometry_msgs:: PointStamped right_toe = findToe(right_leg);
            ROS_INFO("RIGHT LEG");
            //geometry_msgs:: PointStamped right_shin = findShin(right_leg);
//             float distance = sqrt(pow(old_right_toe.point.x - right_toe.point.x, 2) + pow(old_right_toe.point.y - right_toe.point.y, 2) + pow(old_right_toe.point.z - right_toe.point.z, 2));
//             addedDistancesRight += std::abs(distance);

//             old_right_toe = right_toe;
            
            if (!kFilter_right->isInitializated()) {
                ROS_INFO("INIT RIGHT FILTER");
                std::vector<double> toeRight;
                toeRight.push_back(right_toe.point.x);
                toeRight.push_back(right_toe.point.y);
                toeRight.push_back(right_toe.point.z);
                toeRight.push_back(0);
                toeRight.push_back(0);
                toeRight.push_back(0);
                previous_T_right = ros::Time::now();
                previous_T_right_seq = ros::Time::now();
                x_hats[1] = toeRight;
                
                if (!kFilter_right->configure(toeRight)) ROS_ERROR("KalmanFilter for the right Toe couldn't configure!");
            }
            else{
                ROS_INFO("RIGHT TOE Measured");
                std::vector<double> data_in_r, data_out_r, data_out_r_seq;
                data_in_r.push_back(right_toe.point.x);
                data_in_r.push_back(right_toe.point.y);
                data_in_r.push_back(right_toe.point.z);
                ros::Time now_right = ros::Time::now();
                double delta_t_right = (now_right - previous_T_right).toSec();
                
                
                kFilter_right->update(data_in_r, data_out_r, delta_t_right, true, R_loc);
                previous_T_right = now_right;

                geometry_msgs::PointStamped msg = vecToPointStamped(data_out_r, right_toe.header);
                pub_right_toe_kf.publish(msg);

                mtx_right.lock();
                if(kFilter_LSR_right->isInitializated()) {
                    double delta_t_right_seq = (now_right - previous_T_right_seq).toSec();
                    kFilter_LSR_right->update(data_in_r, data_out_r_seq, delta_t_right_seq, true, R_loc);
                    x_hats[1] = data_out_r_seq;
                    previous_T_right_seq = now_right;
                    
                    geometry_msgs::PointStamped msg_seq = vecToPointStamped(data_out_r_seq, right_toe.header);
                    pub_right_toe_seq.publish(msg_seq);
                }
                right_init = true;
                mtx_right.unlock();
            }
            pub_right_leg.publish(right_leg);
            pub_right_toe.publish(right_toe);
        }
        //Should only be true
//         ROS_INFO("The average distance in the LEFT toe is %fm", addedDistancesLeft/iterations);
//         ROS_INFO("The average distance in the RIGHT toe is %fm", addedDistancesRight/iterations);
    ros::Time end = ros::Time::now();
    //ROS_INFO("THIS CALLBACK TOOK %f SECONDS", (end - start).toSec());

    }
}

//side == 0 --> left leg; 1 --> right_leg
std::vector<double> shinToToe(leg_tracker::LegMsg leg, std::vector<double> toe_est) {
    std::vector<double> out;
    std::vector<double> x_hat = toe_est;
    double azimuth, inclination;

    //laser scanner at height of 0.178 according to tf
    double x_dist = x_hat[0] - leg.position.x;
    double y_dist = x_hat[1] - leg.position.y;
    double z_dist = x_hat[2] - SHIN_HEIGHT;
    
    double r = std::sqrt( std::pow(x_dist,2) +  std::pow(y_dist,2) + std::pow(z_dist,2) );
    if (shinToeDist == 0) shinToeDist = r;
    else shinToeDist = (shinToeDist + r) / 2;
    
    if (r != 0) {
        inclination = std::acos( z_dist / r);
        azimuth = std::atan2(y_dist, x_dist);
    }
    else {
        inclination = 0;
        azimuth = 0;
    }
    //Calculate estimated toe position with sphere coordinates
    
    ROS_INFO("R is %.8f Azimuth is %.8f Inclination is %.8f",shinToeDist , azimuth, inclination);
    
    double x_toe = leg.position.x + shinToeDist * std::sin(inclination) * std::cos(azimuth);
    double y_toe = leg.position.y + shinToeDist * std::sin(inclination) * std::sin(azimuth);
    double z_toe = SHIN_HEIGHT + shinToeDist * std::cos(inclination);
    ROS_INFO("Estimation is %.8f, %.8f, %.8f; estimate was %.8f,%.8f,%.8f,",x_toe,y_toe,z_toe, x_hat[0], x_hat[1], x_hat[2]);
    out.push_back(x_toe);
    out.push_back(y_toe);
    out.push_back(z_toe);
    
    return out;
}

void lsr_cb(leg_tracker::PersonMsg msg) {
    //ROS_INFO("Laser callback, received msg is x : %f, y : %f ! ", msg.leg1.position.x , msg.leg1.position.y);

    //ROS_INFO("Laser passed p1 completed");
    std::vector<double> x_left_in, x_right_in;
    std::vector<double> x_left_out(3), x_right_out(3);
    
    //new prediction for x need here?
    leg_tracker::LegMsg right_leg, left_leg;
    
    if (msg.leg1.position.y > msg.leg2.position.y) {
        left_leg = msg.leg1;
        right_leg = msg.leg2;
    }
    else {
        left_leg = msg.leg2;
        right_leg = msg.leg1;
    }
    
    //ROS_INFO("Left Values x: %.8f, y: %.8f z: %.8f", x_left_in[0], x_left_in[1], x_left_in[2]); 
    //Update here load R matrix? use seperate Kalmans?
      
    //geometry_msgs::PointStamped left_msg = vecToPointStamped(x_left_in, msg.header);
    //geometry_msgs::PointStamped right_msg = vecToPointStamped(x_right_in, msg.header);
    if(!(kFilter_LSR_left->isInitializated()) && left_init){
        mtx_left.lock();
        ROS_INFO("INIT KALMAN SEQ LEFT");
        std::vector<double> est_left(3);
        double delta_t_left = (ros::Time::now() - previous_T_left).toSec();
        kFilter_left->computePrediction(est_left, delta_t_left);
        if(!kFilter_LSR_left->configure(est_left)) ROS_ERROR("Seq KalmanFilter for the left Toe couldn't configure!");
        previous_T_left_seq = ros::Time::now();
        mtx_left.unlock();
    }
    else if(kFilter_LSR_left->isInitializated() && left_init) {
        mtx_left.lock();
        std::vector<double> est_left(3); 
        double delta_t_left = (ros::Time::now() - previous_T_left_seq).toSec();
        kFilter_LSR_left->computePrediction(est_left, delta_t_left);
        std::vector<double> toe_est = shinToToe(left_leg, est_left);
        std::cout<< "Measurement vecotr left is: " << toe_est.size() << "x : " << toe_est[0] << " y: " << toe_est[1] << " z: " << toe_est[2] << std::endl;
        kFilter_LSR_left->update(toe_est, x_left_out, delta_t_left, true);
        x_hats[0] = x_left_out;
        geometry_msgs::PointStamped left_msg = vecToPointStamped(x_left_out, msg.header);
        previous_T_left_seq = ros::Time::now();
        pub_left_toe_seq.publish(left_msg);
        mtx_left.unlock();
    }
    if(!(kFilter_LSR_right->isInitializated()) && right_init){
        mtx_left.lock();
        ROS_INFO("INIT KALMAN SEQ RIGHT");
        std::vector<double> est_right(3);
        double delta_t_right = (ros::Time::now() - previous_T_right).toSec();
        kFilter_right->computePrediction(est_right, delta_t_right);
        if(!kFilter_LSR_right->configure(est_right)) ROS_ERROR("Seq KalmanFilter for the right Toe couldn't configure!");
        previous_T_right_seq = ros::Time::now();
        mtx_left.unlock();
    }
    else if(kFilter_LSR_right->isInitializated() && right_init) {
        mtx_right.lock();
        std::vector<double> est_right(3); 
        double delta_t_right = (ros::Time::now() - previous_T_right_seq).toSec();
        kFilter_LSR_right->computePrediction(est_right, delta_t_right);
        std::vector<double> toe_est = shinToToe(right_leg, est_right);
        std::cout<< "Measurement vecotr right is: " << toe_est.size() << "x : " << toe_est[0] << " y: " << toe_est[1] << " z: " << toe_est[2] << std::endl;
        kFilter_LSR_right->update(toe_est, x_right_out, delta_t_right, true);
        x_hats[1] = x_right_out;
        geometry_msgs::PointStamped right_msg = vecToPointStamped(x_right_out, msg.header);
        previous_T_right_seq = ros::Time::now();
        pub_right_toe_seq.publish(right_msg);
        mtx_right.unlock();
    }    
}

bool fromStdVectorToEigenMatrix(std::vector<double>& in, Eigen::MatrixXd& out, int rows, 
							     int columns, std::string matrix_name) {
  if (in.size() != rows * columns || in.size() == 0) { ROS_ERROR("%s is not valid!", matrix_name.c_str()); return false; }
  out.resize(rows, columns);
  std::vector<double>::iterator it = in.begin();
  for (int i = 0; i < rows; ++i)
  {
    for (int j = 0; j < columns; ++j) 
    {
      out(i, j) = *it;
      ++it;
    }
  }
  return true;
}

int main (int argc, char** argv) {
    // Initialize ROS
    ros::init (argc, argv, "toe_detection");
    ros::NodeHandle nh;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);


    try {
        transformStamped = tfBuffer.lookupTransform("base_link", "camera_legs_depth_optical_frame", ros::Time(0), ros::Duration(2));
    } catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
    }
    
    kFilter_left = new KalmanFilter();
    kFilter_right = new KalmanFilter();
    kFilter_LSR_left = new KalmanFilter();
    kFilter_LSR_right = new KalmanFilter();
    
    int m = 0;
    std::vector<double> R_vec;
    if(nh.getParam("/toe_detection/KalmanFilter/m",m) && nh.getParam("/toe_detection/2R",R_vec)) {
     R_loc = Eigen::MatrixXd(m,m);
     fromStdVectorToEigenMatrix(R_vec, R_loc, m, m, "R_loc");
     sequentiell = true;
     std::cout << "R_vec has dimensions " << R_vec.size() << " m is " << m << "\n";
     std::cout << "R_loc is " << R_loc << std::endl;
    }
    else {
     ROS_ERROR("Parameters for sequentiell Kalman not loaded");
     ROS_INFO("M loaded ? %d; R loaded ? %d", nh.getParam("/toe_detection/KalmanFilter/m",m), nh.getParam("/toe_detection/KalmanFilter/2R",R_vec));
    }
    
    int a = 1, b = 2;
    int c = a + b;
    std::cout << "\n blurb c is "<< c << std::endl;
    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("/camera_legs/depth_registered/points", 1, cloud_cb);
    ros::Subscriber sub_lsr = nh.subscribe ("/leg_detection/people_msg_stamped", 1, lsr_cb);

    pub_left_leg = nh.advertise<sensor_msgs::PointCloud2>("left_leg", 1);
    pub_right_leg = nh.advertise<sensor_msgs::PointCloud2>("right_leg", 1);
    //pub_left_leg = nh.advertise<geometry_msgs::PointStamped>("left_leg", 1);
    //pub_right_leg = nh.advertise<geometry_msgs::PointStamped>("right_leg", 1);
    pub_left_toe = nh.advertise<geometry_msgs::PointStamped>("left_toe", 1);
    pub_right_toe = nh.advertise<geometry_msgs::PointStamped>("right_toe", 1);
    
    pub_left_toe_kf = nh.advertise<geometry_msgs::PointStamped>("left_toe_kf",1);
    pub_right_toe_kf = nh.advertise<geometry_msgs::PointStamped>("right_toe_kf",1);
    
    pub_left_toe_seq = nh.advertise<geometry_msgs::PointStamped>("left_toe_seq",1);
    pub_right_toe_seq = nh.advertise<geometry_msgs::PointStamped>("right_toe_seq",1);

    ros::spin();
    return 0;
}
