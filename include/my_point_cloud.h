#ifndef MY_POINT_CLOUD_H
#define MY_POINT_CLOUD_H

#include <vector>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef typename PointCloud::Ptr PointCloudPtr;


#endif /*MY_POINT_CLOUD_H*/
