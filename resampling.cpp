#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

using namespace std;

int
main (int argc, char** argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ> ());
    // Load bun0.pcd -- should be available with the PCL archive in test 
    pcl::io::loadPCDFile ("/home/gordon/feelEnvironment/data/zhuzi-noimu.pcd", *cloud);
    cout << "input point number : " << cloud->points.size() << endl;

//     cloud = cloud2;
//     cout << "cloud size : " << cloud->points.size() << endl;
//     cout << "cloud2 size : " << cloud->points.size() << endl;
    
    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
    
    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.3);

    // Reconstruct
    mls.process (mls_points);

    // Save output
    pcl::io::savePCDFile ("/home/gordon/feelEnvironment/data/zhuzi-noimu-mls.pcd", mls_points);
}
