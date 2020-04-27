#include <string>
#include <iostream>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <Eigen/Geometry>
#include <Eigen/Core>

void pcd2txt(pcl::PointCloud<pcl::PointXYZI>::Ptr pcdPtr)
{
    /*------------------ output pointcloud xyz as txt ----------------*/
    /*----------------------------------------------------------------*/
    std::ofstream xyz_txt("/home/gordon/feelEnvironment/data/segByCloudCompare/raw_pointcloud/pointcloud_xyz.txt", std::ios::out); 
    for(int i = 0; i < pcdPtr->points.size(); i++)
    {
        xyz_txt << std::fixed << std::setprecision(6) << pcdPtr->points[i].x << ' ' << pcdPtr->points[i].y << ' ' << pcdPtr->points[i].z << std::endl;
    }
    xyz_txt.close();
}


void txt2pcd(pcl::PointCloud<pcl::PointXYZ>::Ptr pcdPtr){
    std::ifstream fin("/home/gordon/feelEnvironment/data/segByCloudCompare/326room.txt", std::ios::in);
    int row;
    fin >> row;
    std::cout << "row size = " << row << std::endl;
    
    double temp;
    pcl::PointXYZ point;
    for(int i = 0; i < row; i++){
        fin >> temp;
        point.x = temp;
        
        fin >> temp;
        point.y = temp;
        
        fin >> temp;
        point.z = temp;
        
        pcdPtr->points.push_back(point);
    }
    
    if(row == pcdPtr->size())
        std::cout << "Success!" << std::endl;
    
}

int main (int argc, char** argv)
{
//     pcd to txt
    // pcl::PCDReader reader;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    
    // std::string fileLocation = "/home/gordon/feelEnvironment/data/segByCloudCompare/raw_pointcloud/xyz02.pcd";
    // reader.read(fileLocation,*cloud);
    
    // std::cout << "ori_pointcloud size = "<< cloud->points.size() << std::endl;
    // std::cout << "width = " << cloud->width << "; height = " << cloud->height << std::endl;
    // std::cout << "sensor origin : " << cloud->sensor_origin_ << std::endl;
    // std::cout << "sensor orientation : " << cloud->sensor_orientation_.coeffs() << std::endl;
    
    // pcd2txt(cloud);


    // txt to pcd
    pcl::PCDWriter writer;
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr segCloud(new pcl::PointCloud<pcl::PointXYZ>);
    txt2pcd(segCloud);
    segCloud->width = segCloud->size();
    segCloud->height = 1;
    
    writer.write("/home/gordon/feelEnvironment/data/segByCloudCompare/xyz326room.pcd", *segCloud);
    

    return 0;
}
