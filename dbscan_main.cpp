#include "dbscan.h"
#include <iostream>

using namespace std;

// Used after DBSCAN to remove noise
void extract_from_indices(PointCloudPtr input_pc, vector<int> &label, PointCloudPtr output_pc)
{
    for (int i = 0; i < label.size(); i++)
    {
        if(label[i] > 0)
            output_pc->points.push_back(input_pc->points[i]);
    }
}

int main()
{
    
    pcl::PCDReader reader;
    PointCloudPtr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    reader.read("/home/gordon/feelEnvironment/data/zhuzi-noimu.pcd",*cloud);
    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " points." << std::endl;
    
    pcl::visualization::CloudViewer viewer1("01 Input pointcloud");
    viewer1.showCloud(cloud,"1");
    while (!viewer1.wasStopped ())
    {}

    vector<int> labels;
    time_t start, stop;
    
    start = time(NULL);
    int num = dbscan(cloud, labels, 0.3, 60);
    stop = time(NULL);
    
    cout << "cost time : " << stop-start << endl;
    cout<<"cluster size is "<<num<<endl;
    
    PointCloudPtr DB_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    extract_from_indices(cloud, labels, DB_cloud);
    std::cout << "PointCloud after filtering has: " << DB_cloud->points.size () << " points." << std::endl;
    

    
    for(int i = 0; i < (int)cloud->points.size(); i++){
        std::cout<<"Point( "<< cloud->points[i].x<< ", " <<cloud->points[i].y<< ", "<< cloud->points[i].z<< " ): "<<labels[i]<<std::endl;
    }
    
    pcl::visualization::CloudViewer viewer2("02 DBSCAN pointcloud");
    viewer2.showCloud(DB_cloud,"1");
    while (!viewer2.wasStopped ())
    {}


    
    return 0;
}

