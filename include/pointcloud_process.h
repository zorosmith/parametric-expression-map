#ifndef POINTCLOUD_PROCESS_H
#define POINTCLOUD_PROCESS_H

#include "my_point_cloud.h"
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

using namespace pcl;
using namespace std;

/*-----------------------------------------------------------------------*/
/*------------------------- pointcloud filter ---------------------------*/

// add statistical filter to remove outliers
//     float statistical_mean = atof( pr.getData( "statistical_mean" ).c_str());
//     float statistical_stddev = atof( pr.getData( "statistical_stddev" ).c_str());
//     
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_statistical_filtered (new pcl::PointCloud<pcl::PointXYZ>);
//     pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//     sor.setInputCloud(cloud_voxelgrid_filtered);
//     sor.setMeanK(statistical_mean);
//     sor.setStddevMulThresh(statistical_stddev);
//     sor.filter(*cloud_statistical_filtered);
//     std::cout << "PointCloud after Statistical filtering has: " << cloud_statistical_filtered->points.size ()  << " points." << std::endl;











/*-----------------------------------------------------------------------*/
/*------------------------- pointcloud segmentation ---------------------*/
std::vector<pcl::PointIndices> Region_Growing (PointCloudPtr input_pc, int min_num)
{
    pcl::search::Search<PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointXYZ> > (new pcl::search::KdTree<PointXYZ>);
    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    normal_estimator.setInputCloud (input_pc);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (min_num);
    reg.setMaxClusterSize (1000000);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (30);
    reg.setInputCloud (input_pc);
    //reg.setIndices (indices);   对经过直通滤波的部分点云进行处理，不需要对全局点云进行处理
    reg.setInputNormals (normals);
    reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
    reg.setCurvatureThreshold (0.5);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);
    
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.getColoredCloud ();
    pcl::visualization::CloudViewer viewer_RG ("RegionGrowing");
    viewer_RG.showCloud(colored_cloud);
    while (!viewer_RG.wasStopped ())
    {
    }
    
    return clusters;
}


void Euclidean_Cluster_Extraction(PointCloudPtr input_pc, vector<pcl::PointIndices> &output_indices, float toler)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (input_pc);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (toler); // 2cm
    ec.setMinClusterSize (500);
    ec.setMaxClusterSize (25000000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (input_pc);
    ec.extract (output_indices);
    
}


/*-----------------------------------------------------------------------*/
/*--------------------- pointcloud other processing ---------------------*/
void NormalEstimation (PointCloudPtr input_pointcloud, pcl::PointCloud<pcl::Normal>::Ptr normals)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setViewPoint (std::numeric_limits<float>::max (), std::numeric_limits<float>::max (), std::numeric_limits<float>::max ());
    normal_estimator.setInputCloud (input_pointcloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*normals);
}


// Extract the plane points from the input pointcloud according to the input indices
void Extract_Indices (PointCloudPtr input_pc, PointCloudPtr output_pc, pcl::PointIndices::Ptr input_indices,
                      bool Neg_flag, bool Keep_flag = false)
{
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (input_pc);
    extract.setIndices (input_indices);
    extract.setNegative (Neg_flag); // if false, get the plane points; if true, get points except for plane points
    extract.setKeepOrganized (Keep_flag); // keep the order of indices; the order would be changed once the poincloud being trimmed 
    extract.filter (*output_pc);
    
}

// project the point cloud according to the model
void Project_Inliers(PointCloudPtr input_pc, ModelCoefficients::Ptr plane_coef, PointCloudPtr output_pc)
{
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType (pcl::SACMODEL_PLANE);
    proj.setInputCloud (input_pc);
    proj.setModelCoefficients (plane_coef);
    proj.filter (*output_pc);
    
}


// get the outline of the point cloud
void Concave_Hull(PointCloudPtr input_pc, PointCloudPtr output_pc, float alpha)
{
    pcl::ConcaveHull<pcl::PointXYZ> chull;
//  pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (input_pc);
    chull.setAlpha (alpha);   // the smaller the more detailed the hull
    chull.reconstruct (*output_pc);
}


void getPointXYZ(pcl::PointCloud<pcl::PointNormal> &input_pointnormal, PointCloudPtr output_pc)
{
    int num = input_pointnormal.size();
    for(int i = 0; i < num; i++)
    {
        PointT point;
        point.x = input_pointnormal.points[i].x;
        point.y = input_pointnormal.points[i].y;
        point.z = input_pointnormal.points[i].z;
        output_pc->points.push_back(point);
    }
    
}

/*-----------------------------------------------------------------------*/
/*------------------------- pointcloud reconstruct ---------------------*/     
void Moving_Least_Square(PointCloudPtr input_pc, PointCloudPtr output_pc, int polynomialOrder, float searchRadius)
{
    

    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    
    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (input_pc);
    mls.setPolynomialOrder (polynomialOrder);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (searchRadius);

    // Reconstruct
    mls.process (mls_points);
    
    getPointXYZ(mls_points, output_pc);
}


#endif