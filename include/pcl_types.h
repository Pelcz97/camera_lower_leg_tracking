 
#ifndef _PCL_TYPES_H
#define _PCL_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZRGB Point;

typedef pcl::PointCloud<Point> Cloud;
typedef Cloud::Ptr Cloud_ptr;
typedef Cloud::ConstPtr Cloud_cptr;

typedef pcl::Normal Normal;
typedef pcl::PointCloud<Normal> Normals;
typedef Normals::Ptr Normals_ptr;
typedef Normals::ConstPtr Normals_cptr;

typedef std::vector<int> Indices;
typedef boost::shared_ptr<Indices> Indices_ptr;
typedef boost::shared_ptr<Indices const> Indices_cptr;


#endif//_PCL_TYPES_H
