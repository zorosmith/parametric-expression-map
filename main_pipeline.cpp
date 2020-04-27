#include "pointcloud_process.h"
#include "dbscan.h"
#include "accuracy_measure.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <cmath>
// #include <ctime>
#include <boost/thread/thread.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry> 

#include <opencv2/opencv.hpp> 

#define pi 3.1415926

using namespace std;
using namespace pcl;
// using namespace Eigen;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

// clock
time_t start, stop;

// read parameters from parameter.txt
extern ParameterReader pr;

string file_location = pr.getData("pcd_location");           // main. The path to the pcd file
float leafsize = atof( pr.getData( "leaf_size" ).c_str());   // main. The leaf size of the VoxelGrid 

float euler_tolerance = atof( pr.getData( "cluster_tolerance" ).c_str());   // main. EuclideanCluster

float sfMean = atof( pr.getData( "statistical_mean" ).c_str());   //main.statistical filter
float sfStddev = atof( pr.getData( "statistical_stddev" ).c_str());   //main.statistical filter

float DB_eps = atof( pr.getData( "EPS" ).c_str());     // main. DBSCAN
int DB_min = atoi( pr.getData( "MIN_NUM" ).c_str());   // main. DBSCAN

int polynomialOrder = atoi( pr.getData( "Polynomial_Order" ).c_str());   // main. Moving_Least_Square
float searchRadius = atof( pr.getData( "Search_Radius" ).c_str());       // main. Moving_Least_Square

float cluster_num = atof( pr.getData( "cluster_num" ).c_str());      // main. parameter for RegionGrowing
int MinClusterSize = atoi( pr.getData( "MinClusterSize" ).c_str());  //main. parameter for RegionGrowing
float rgSmooth = atof( pr.getData( "rgSmooth" ).c_str());        // main. parameter for RegionGrowing
float rgCurvature = atof( pr.getData( "rgCurvature" ).c_str());  // main. parameter for RegionGrowing

float dist_thres = atof( pr.getData( "DistanceThreshold" ).c_str());   // main. parameter for RANSAC plane
float remain_scale = atof( pr.getData( "remain_scale" ).c_str());      // main. parameter for RANSAC plane

float alpha = atof( pr.getData( "setAlpha" ).c_str());  // main. Concave_Hull

float epsilon = atof( pr.getData( "Epsilon" ).c_str());  // getPlaneApproxVertices. 


// visualizer
void visualizer_window(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, string window_name, bool flag = true)
{
  if (flag == false)
  {
    return;
  }
  pcl::visualization::CloudViewer viewer_port(window_name);
  viewer_port.showCloud(input_cloud,"1");
  while (!viewer_port.wasStopped ())
  {}
}


bool sortEuclideanIndices(const pcl::PointIndices& indice1 , const pcl::PointIndices& indice2)
{
    return(indice1.indices.size() > indice2.indices.size());
}


void averagePointxyz(const pcl::PointXYZ input_pt1, const pcl::PointXYZ input_pt2 , pcl::PointXYZ& output_pt3)
{
    output_pt3.x = (input_pt1.x + input_pt2.x)/2 ;
    output_pt3.y = (input_pt1.y + input_pt2.y)/2 ;
    output_pt3.z = (input_pt1.z + input_pt2.z)/2 ;
}


// distance of point to line
// Input : point(x,y,z)
double DistanceOfPointToLine(Eigen::Vector3d l1, Eigen::Vector3d l2, Eigen::Vector3d s) 
{ 
	double ab = sqrt(pow((l1(0)-l2(0)), 2.0) + pow((l1(1)-l2(1)), 2.0) + pow((l1(2)-l2(2)), 2.0));
	double as = sqrt(pow((l1(0)-s(0)), 2.0) + pow((l1(1)-s(1)), 2.0) + pow((l1(2)-s(2)), 2.0));
	double bs = sqrt(pow((s(0)-l2(0)), 2.0) + pow((s(1)-l2(1)), 2.0) + pow((s(2)-l2(2)), 2.0));
	double cos_A = (pow(as, 2.0) + pow(ab, 2.0) - pow(bs, 2.0)) / (2 * ab*as);
	double sin_A = sqrt(1 - pow(cos_A, 2.0));
	return as*sin_A; 
}

// distance of point to line
// Input : point(x,y)
double DistanceOfPointToLine(cv::Point &al, cv::Point &bl, cv::Point &cp) 
{ 
    double ab = sqrt(pow((al.x - bl.x), 2.0) + pow((al.y - bl.y), 2.0));
    double ac = sqrt(pow((al.x - cp.x), 2.0) + pow((al.y - cp.y), 2.0));
    double bc = sqrt(pow((cp.x - bl.x), 2.0) + pow((cp.y - bl.y), 2.0));
    double cos_A = (pow(ac, 2.0) + pow(ab, 2.0) - pow(bc, 2.0)) / (2 * ab*ac);
    double sin_A = sqrt(1 - pow(cos_A, 2.0));
    return ac*sin_A; 
}

// calculate the angle between two lines
std::vector<int> angle_deviation(std::vector< pcl::ModelCoefficients > &line_dir, int angle_gap)
{
    std::vector<int> paral_indx;
    for(int i = 0; i < line_dir.size(); i++)
    {
        pcl::ModelCoefficients line1 = line_dir[i];
        for(int j = i+1; j < line_dir.size(); j++)
        {
            pcl::ModelCoefficients line2 = line_dir[j];
            double cos_numerator = abs(line1.values[3]*line2.values[3] + line1.values[4]*line2.values[4] + line1.values[5]*line2.values[5]);
            double cos_denominator_L = sqrt(pow((line1.values[3]), 2.0) + pow((line1.values[4]), 2.0) + pow((line1.values[5]), 2.0));
            double cos_denominator_R = sqrt(pow((line2.values[3]), 2.0) + pow((line2.values[4]), 2.0) + pow((line2.values[5]), 2.0));
            double cos_angle = cos_numerator / (cos_denominator_L * cos_denominator_R);
            std::cout << "cos angle between two lines : " << cos_angle << std::endl;
            double thres = cos(angle_gap * pi / 180);  // 判别两直线是否平行的阈值，8°
            if(cos_angle >= thres)
            {
                paral_indx.push_back(i);
                paral_indx.push_back(j);
            }
        }

    }
    for(int k = 0 ; k < paral_indx.size()/2 ; k++)
    {
        std::cout << "the index of parallel lines : " << paral_indx[2*k] << " " << paral_indx[2*k+1] << std::endl;
    }

    return paral_indx;
}





/*//若点a大于点b,即点a在点b顺时针方向,返回true,否则返回false
bool PointCmp(cv::Point &a, cv::Point &b, cv::Point &center)
{
    if (a.x >= 0 && b.x < 0)
        return true;
    if (a.x == 0 && b.x == 0)
        return a.y > b.y;
    //向量OA和向量OB的叉积
    int det = (a.x - center.x) * (b.y - center.y) - (b.x - center.x) * (a.y - center.y);
    if (det < 0)
        return true;
    if (det > 0)
        return false;
    //向量OA和向量OB共线，以距离判断大小
    int d1 = (a.x - center.x) * (a.x - center.x) + (a.y - center.y) * (a.y - center.y);
    int d2 = (b.x - center.x) * (b.x - center.y) + (b.y - center.y) * (b.y - center.y);
    return d1 > d2;
}

void ClockwiseSortPoints(vector< cv::Point> &vPoints)
{
    //计算重心
    cv::Point center;
    double x = 0,y = 0;
    for (int i = 0; i < vPoints.size(); i++)
    {
        x += vPoints[i].x;
        y += vPoints[i].y;
    }
    center.x = (int)x/vPoints.size();
    center.y = (int)y/vPoints.size();
    
    //冒泡排序
    for(int i = 0;i < vPoints.size() - 1;i++)
    {
        for (int j = 0;j < vPoints.size() - i - 1;j++)
        {
            if (PointCmp(vPoints[j],vPoints[j+1],center))
            {
                cv::Point tmp = vPoints[j];
                vPoints[j] = vPoints[j + 1];
                vPoints[j + 1] = tmp;
            }
        }
    }
    
    for(int n = 0; n < vPoints.size(); n++)
    {
        printf ("No %d : %d, %d\n", n, vPoints.at(n).x, vPoints[n].y);

        
    }
    cout << "After filtering, we have : " << vPoints.size() << endl;
} */ 





// calculate the length of line
// change this function to be a oveload function of Normalize
double line_length(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    pcl::PointXYZ point1, point2;
    point1 = cloud.points[0];
    point2 = cloud.points[1];
    double len = sqrt(pow((point1.x - point2.x), 2.0) + pow((point1.y - point2.y), 2.0) + pow((point1.z - point2.z), 2.0));
    if(len <= 0){
        std::cout << "error in calculate the length of the parallel lines" << std::endl;
    }
    return len;
}

Eigen::Matrix3d Rotational_Matrix(double agl, Eigen::Vector3d axis)
{
    Eigen::Matrix3d output;
    axis.normalize();
    
    output(0,0) = cos(agl) + pow(axis(0),2)*(1-cos(agl));
    output(0,1) = axis[0]*axis[1]*(1-cos(agl))-axis[2]*sin(agl);
    output(0,2) = axis[1]*sin(agl)+axis[0]*axis[2]*(1-cos(agl));
    
    output(1,0) = axis[2]*sin(agl)+axis[0]*axis[1]*(1-cos(agl));
    output(1,1) = cos(agl)+pow(axis[1],2)*(1-cos(agl));
    output(1,2) = -axis[0]*sin(agl)+axis[1]*axis[2]*(1-cos(agl));
    
    output(2,0) = -axis[1]*sin(agl)+axis[0]*axis[2]*(1-cos(agl));
    output(2,1) = axis[0]*sin(agl)+axis[1]*axis[2]*(1-cos(agl));
    output(2,2) = cos(agl)+pow(axis[2],2)*(1-cos(agl));
    
    return output;
}


    
// output rotation matrix
Eigen::Matrix3d normals_rotate(Eigen::Vector3d in_begin, Eigen::Vector3d in_end)
{
    Eigen::Matrix3d output;
    Eigen::Vector3d rotationAxis = in_begin.cross(in_end);
    
    // double alpha = 1.0, actually alpha is equal to 1.0000xxx
    double alpha = in_begin.dot(in_end)/(in_begin.norm()*in_end.norm());
    double rotationAngle = acos((alpha > 0.99) ? 0.99 : (alpha < -0.99) ? -0.99 : alpha);
    output = Rotational_Matrix(rotationAngle, rotationAxis);
    
    // Test
    //cout << "rotation matrix3d " << endl << output << endl;;
    //cout << "the result after rotating the in_begin" << endl << output*in_begin << endl;;
    
    return output;    
}


// calibrate the plane coordinate
void calibrate_coordinate(PointCloudXYZ::Ptr input, vector<Eigen::Vector3d>& output, Eigen::Matrix3d rotation)
{
    for(int i = 0; i < input->points.size(); i++)
    {
        Eigen::Vector3d p1, p2;
        for(int j=0; j < 3; j++)
        {
            p1(j) = input->points[i].data[j];            
        }
        p2 = rotation*p1;
        output.push_back(p2);
    }
}


    
// approx contour
// https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
// 1 point; 2 point indices
tuple<vector< Eigen::Vector3d >, vector<int>> DouglasPeucker( vector< Eigen::Vector3d > input_list, 
                                                       double epsilon, 
                                                       int begin_idx = 0)
{
    vector< Eigen::Vector3d > plane_contour;
    
    vector< Eigen::Vector3d > output_list;
    vector<int> output_index;
    float dmax = 0.0;
    int index = 0;
    int len = input_list.size();
    for (int i = 1; i < len-1; i++)
    {
        double dist_ptol = DistanceOfPointToLine(input_list[0], input_list[len-1], input_list[i]);
        if (dist_ptol > dmax)
        {
            index = i;
            dmax = dist_ptol;
        }
    }
    
    if(dmax > epsilon)
    {
        vector< Eigen::Vector3d > part1, part2;
        part1.assign(input_list.begin(), input_list.begin()+index+1);
        part2.assign(input_list.begin()+index, input_list.begin()+len);
        tuple<vector< Eigen::Vector3d >, vector<int>> output1 = DouglasPeucker(part1, epsilon, begin_idx);
        tuple<vector< Eigen::Vector3d >, vector<int>> output2 = DouglasPeucker(part2, epsilon, begin_idx+index);
        
        vector< Eigen::Vector3d > output_list1, output_list2;
        vector< int > output_index1, output_index2;
        std::tie(output_list1, output_index1) = output1;
        std::tie(output_list2, output_index2) = output2;
        
        for(int j = 0; j<output_list1.size(); j++)
        {
            output_list.push_back(output_list1[j]);
            output_index.push_back(output_index1[j]);
        }
        
        for(int k = 1; k<output_list2.size(); k++)
        {
            output_list.push_back(output_list2[k]);
            output_index.push_back(output_index2[k]);            
        }
        
    }
    else
    {
        output_list.push_back(input_list[0]);
        output_list.push_back(input_list[len-1]);
        
        output_index.push_back(begin_idx);
        output_index.push_back(begin_idx+len-1);
    }
    
    tuple< vector< Eigen::Vector3d >, vector<int> > output(output_list, output_index);
    return output;
}


void getPlaneApproxVertices(PointCloudXYZ::Ptr input_convexHull,
                             ModelCoefficients::Ptr input_coefficients,
                             vector<pcl::PointXYZ>& output_vertices)
{
    int num_contour = input_convexHull->points.size();
    //cout << "Now, we have PlaneApproxVertices : " << num_contour << endl;
    vector< cv::Point > uv_contour;
    
    // begin: the original normal of plane;
    // end : (0,0,1), Z axis
    Eigen::Vector3d begin_vec;
    for(int i = 0; i < 3; i++)
    {
        begin_vec(i) = input_coefficients->values[i+3];        
    }
    Eigen::Vector3d end_vec(0,0,1);
    
    Eigen::Matrix3d rotation_M = normals_rotate(begin_vec, end_vec);
    vector< Eigen::Vector3d > plane_contour;
    calibrate_coordinate(input_convexHull, plane_contour, rotation_M);
    
    // prepare epsilon for DouglasPeucker algorithm

    tuple<vector< Eigen::Vector3d >, vector<int>> approx_output = DouglasPeucker(plane_contour, epsilon);
    vector< Eigen::Vector3d > approx_contour; // contour points,x and y
    vector<int> approx_index;  // contour points, index
    std::tie(approx_contour, approx_index) = approx_output;
    
    //cout << "after approx the contour : " << approx_index.size()<< endl;
//     for(int n = 0; n < approx_contour.size(); n++)
//     {
//         printf ("No %d : %f, %f\n", n, approx_contour.at(n)(0), approx_contour[n](1));
//         printf ("%d, \n", approx_index[n]);
//     }
    
    for(int i = 0; i < approx_contour.size(); i++)
    {
        output_vertices.push_back(input_convexHull->points[approx_index[i]]);
    }
    cout << "Finish approxing plane contour" << endl;
    
}

void momentOfInertia(PointCloudXYZ::Ptr input_pc, std::vector<pcl::PointXYZ>& point_singleGroup, bool is_viz)
{
    point_singleGroup.clear();

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud (input_pc);
    feature_extractor.compute ();

    std::vector <float> moment_of_inertia;
    std::vector <float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;

    feature_extractor.getMomentOfInertia (moment_of_inertia);
    feature_extractor.getEccentricity (eccentricity);
    feature_extractor.getAABB (min_point_AABB, max_point_AABB);
    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    feature_extractor.getEigenValues (major_value, middle_value, minor_value);
    feature_extractor.getEigenVectors (major_vector, middle_vector, minor_vector);
    feature_extractor.getMassCenter (mass_center);

    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
//    Eigen::Quaternionf quat (rotational_matrix_OBB);

    // computer OBB`s 8 points
    Eigen::Vector3f p1 (min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p2 (min_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p3 (max_point_OBB.x, min_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p4 (max_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p5 (min_point_OBB.x, max_point_OBB.y, min_point_OBB.z);
    Eigen::Vector3f p6 (min_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p7 (max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    Eigen::Vector3f p8 (max_point_OBB.x, max_point_OBB.y, min_point_OBB.z);

    p1 = rotational_matrix_OBB * p1 + position;
    p2 = rotational_matrix_OBB * p2 + position;
    p3 = rotational_matrix_OBB * p3 + position;
    p4 = rotational_matrix_OBB * p4 + position;
    p5 = rotational_matrix_OBB * p5 + position;
    p6 = rotational_matrix_OBB * p6 + position;
    p7 = rotational_matrix_OBB * p7 + position;
    p8 = rotational_matrix_OBB * p8 + position;

    pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
    pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
    pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
    pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
    pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
    pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
    pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
    pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

    point_singleGroup.push_back(pt1);
    point_singleGroup.push_back(pt2);
    point_singleGroup.push_back(pt3);
    point_singleGroup.push_back(pt4);
    point_singleGroup.push_back(pt5);
    point_singleGroup.push_back(pt6);
    point_singleGroup.push_back(pt7);
    point_singleGroup.push_back(pt8);


    if(is_viz)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (255, 255, 255);
        viewer->addCoordinateSystem (0.1);
        viewer->initCameraParameters ();
        viewer->addPointCloud<pcl::PointXYZ> (input_pc, "sample cloud");
//        viewer->addCube (min_point_AABB.x, max_point_AABB.x, min_point_AABB.y, max_point_AABB.y, min_point_AABB.z, max_point_AABB.z, 1.0, 1.0, 0.0, "AABB");
//        viewer->addCube (position, quat, max_point_OBB.x - min_point_OBB.x, max_point_OBB.y - min_point_OBB.y, max_point_OBB.z - min_point_OBB.z, "OBB");


        viewer->addLine (pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
        viewer->addLine (pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
        viewer->addLine (pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
        viewer->addLine (pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
        viewer->addLine (pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
        viewer->addLine (pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
        viewer->addLine (pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
        viewer->addLine (pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
        viewer->addLine (pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
        viewer->addLine (pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
        viewer->addLine (pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
        viewer->addLine (pt3, pt7, 1.0, 0.0, 0.0, "12 edge");

        while(!viewer->wasStopped())
        {
          viewer->spinOnce (100);
          boost::this_thread::sleep (boost::posix_time::microseconds (100000));
        }
    }
}

// put points from give_sir to get_sir according to the give_sir_indices
void extract_from_indices(PointCloudXYZ::Ptr get_sir, pcl::PointIndices give_sir_indices, PointCloudXYZ::Ptr give_sir)
{
    for (std::vector<int>::const_iterator pit = give_sir_indices.indices.begin (); pit != give_sir_indices.indices.end (); ++pit)
        get_sir->points.push_back (give_sir->points[*pit]); 
}

// put point indices from give_sir to get_sir according to the give_sir_indices
void extract_from_indices(pcl::PointIndices &get_sir, pcl::PointIndices give_sir_indices, pcl::PointIndices give_sir)
{
    for (std::vector<int>::const_iterator pit = give_sir_indices.indices.begin (); pit != give_sir_indices.indices.end (); ++pit)
        get_sir.indices.push_back(give_sir.indices[*pit]);
}


// Used after DBSCAN to remove noise
void extract_from_labels(PointCloudPtr input_pc, vector<int> &label, PointCloudPtr output_pc)
{
    for (int i = 0; i < label.size(); i++)
    {
        if(label[i] > 0)
            output_pc->points.push_back(input_pc->points[i]);
    }
}


pcl::PointXYZ getCenterPoint(vector<pcl::PointXYZ> inputV){
    int nums = inputV.size();
    pcl::PointXYZ result;
    result.x = 0; result.y = 0; result.z = 0;
    
    for (int i = 0; i < nums; i++){
        result.x = result.x + inputV[i].x;
        result.y = result.y + inputV[i].y;
        result.z = result.z + inputV[i].z;
    }
    
    result.x = result.x / nums;
    result.y = result.y / nums;
    result.z = result.z / nums;
    
    return result;
}


int main (int argc, char** argv)
{    
/*----------------- Input pointcloud------------------*/  
/*----------------------------------------------------*/
    pcl::PCDReader reader;
    pcl::PCDReader rgb_reader;
    pcl::PCDWriter writer;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgb_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);


    rgb_reader.read(file_location,*rgb_cloud);
    reader.read (file_location, *cloud);
    std::cout << "Input point cloud has: " << cloud->points.size () << " points." << std::endl;

    pcl::visualization::CloudViewer viewer1("01 Input pointcloud");
    viewer1.showCloud(rgb_cloud,"1");
    while (!viewer1.wasStopped ())
    {}
    
// /*------------------ output pointcloud xyz as txt ----------------*/
// /*----------------------------------------------------------------*/
//     ofstream fout("input_pointcloud_xyz", ios::out); 
//     for(int i = 0; i < cloud->points.size(); i++)
//     {
//         fout << fixed << setprecision(6) << cloud->points[i].x << ' ' << cloud->points[i].y << ' ' << cloud->points[i].z << endl;
//     }
//     fout.close();
    
/*--------------------- Preprocessing --------------------*/ 
/*--------------------- voxel_grid -----------------------*/
    start = clock();
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelgrid_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud (cloud);
    vg.setLeafSize (leafsize, leafsize, leafsize);
    vg.filter (*cloud_voxelgrid_filtered);
    // std::cout << "After VoxselGrid filtering has: " << cloud_voxelgrid_filtered->points.size ()  << " points." << std::endl;

/*--------------------- Preprocessing --------------------*/ 
/*---------------- statistical filter ----------------*/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sf_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    StatisticalFilter(cloud_voxelgrid_filtered, cloud_sf_filtered, sfMean, sfStddev);
    stop = clock();
    std::cout << "After statistical filtering has: " << cloud_sf_filtered->points.size ()  << " points." << std::endl;

    cout << "preprocess cost time : " << double(stop-start)/CLOCKS_PER_SEC << endl;

/*--------------------- Preprocessing --------------------*/ 
/*---------------- EuclideanCluster filter ----------------*/
    std::vector<pcl::PointIndices> euler_cluster;
    Euclidean_Cluster_Extraction(cloud_sf_filtered, euler_cluster, euler_tolerance);
    std::sort( euler_cluster.begin(), euler_cluster.end(), sortEuclideanIndices );
    pcl::PointCloud<pcl::PointXYZ>::Ptr euler_filtered (new pcl::PointCloud<pcl::PointXYZ>);
    PointIndices::Ptr biggest_cluster = boost::make_shared< PointIndices >(euler_cluster[0]);
//    std::cout << "Euler PointIndices : " << *biggest_cluster << std::endl;
    Extract_Indices (cloud_sf_filtered, euler_filtered, biggest_cluster, false);
    std::cout << "After EuclideanCluster filtering has: " << euler_filtered->points.size ()  << " points." << std::endl;

    // visualize pointcloud
    visualizer_window( euler_filtered, "02 preprocess operation");
    
/*------------------ output pointcloud xyz as txt ----------------*/
/*----------------------------------------------------------------*/
//     ofstream eucli_fout("eucli_pointcloud_xyz", ios::out); 
//     for(int i = 0; i < euler_filtered->points.size(); i++)
//     {
//         eucli_fout << fixed << setprecision(6) << euler_filtered->points[i].x << ' ' << euler_filtered->points[i].y << ' ' << euler_filtered->points[i].z << endl;
//     }
//     eucli_fout.close();

    
/*--------------------- Preprocessing --------------------*/
/*---------------------- DBSCAN --------------------------*/
//     vector<int> labels;
//     
//     start = time(NULL);
//     int num = dbscan(euler_filtered, labels, DB_eps, DB_min);
//     stop = time(NULL);
// 
//     cout << "cost time : " << stop-start << endl;
//     cout<<"DBSCAN cluster size is "<<num<<endl;
//     
//     PointCloudPtr DB_cloud (new pcl::PointCloud<pcl::PointXYZ>);
//     extract_from_labels(euler_filtered, labels, DB_cloud);
//     std::cout << "After DBSCAN filtering has: " << DB_cloud->points.size ()  << " points." << std::endl;
//     visualizer_window( DB_cloud, "DBSCAN");
//     cout << "DB_cloud points height and width : " << DB_cloud->height << " , " << DB_cloud->width << endl;
//     DB_cloud->width = DB_cloud->points.size(); DB_cloud->height = 1;
//     writer.write("/home/gordon/feelEnvironment/data/DB_withoutIMU.pcd",*DB_cloud);
    
    
/*--------------------- Preprocessing --------------------*/
/*-------------------  MoveLeastSquare -------------------*/
//     PointCloudPtr MLS_pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
// 
//     Moving_Least_Square(DB_cloud, MLS_pointcloud, polynomialOrder, searchRadius);
//     std::cout << "After Moving_Least_Square filtering has: " << MLS_pointcloud->points.size ()  << " points." << std::endl;
//     visualizer_window( MLS_pointcloud, "MoveLeastSquare");
//     cout << "MLS_pointcloud points height and width : " << MLS_pointcloud->height << " , " << MLS_pointcloud->width << endl;
//     MLS_pointcloud->width = MLS_pointcloud->points.size(); MLS_pointcloud->height = 1;
//     writer.write("/home/gordon/feelEnvironment/data/MLS-withoutIMU.pcd",*MLS_pointcloud);
    
    
/*------------ new pointCloud ptr -----------*/
/*-------------------------------------------*/
    PointCloudPtr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pointcloud_ptr = euler_filtered;
    
/*----------------- Segmentation -----------------*/
/*------------------------------------------------*/
    vector< pcl::PointIndices > seg_indices;
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr regionClusterPtr(new pcl::PointCloud <pcl::PointXYZRGB>);
    seg_indices = Region_Growing (pointcloud_ptr, MinClusterSize, 
                                    rgSmooth, rgCurvature, regionClusterPtr);
    writer.write("/home/gordon/feelEnvironment/data/regionCluter.pcd",*regionClusterPtr);
    cout << "RegionGrowing : " << seg_indices.size() << endl;
    
    // sort cluster_indices; big -> small
    std::sort( seg_indices.begin(), seg_indices.end(), sortEuclideanIndices );
    
    // get the topN pointcloud cluster
    vector<PointCloudXYZ::Ptr> seg_pointcloud;

    for (int i = 0; i < ceil(cluster_num*seg_indices.size()); i++)
    {
        PointCloudXYZ::Ptr seg_pc(new PointCloudXYZ);
        extract_from_indices(seg_pc, seg_indices[i], pointcloud_ptr);
        cout << "RegionGrowing, pointcloud size : " << seg_pc->points.size() << endl;
        seg_pointcloud.push_back(seg_pc);
    }
    cout << "Finish RegionGrowing. TopN pointcloud cluster : " << seg_pointcloud.size() << endl;
    
    
/*--------------------------For every segmented pointcloud ---------------------------*/
/*--------------------- SAC_RANSAC for indices and coefficient Plane -----------------*/

    // containers for point indices and model coefficient of plane
    vector< pcl::PointIndices > seg_plane_indices;   
    vector< pcl::ModelCoefficients > seg_plane_ModelCoefficients;
    
    for(int j = 0; j < seg_pointcloud.size(); j++)
    {
        PointCloudXYZ::Ptr single_pc(new PointCloudXYZ);
        single_pc = seg_pointcloud[j];
        visualizer_window( single_pc, "Each region growing cluster", false);
        
        // Create the segmentation object for the planar model and set all the parameters
        start=clock();
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (dist_thres);    //origin is 0.02m

        int nr_points = (int) single_pc->points.size ();

        while (single_pc->points.size () > remain_scale * nr_points)
        {
            pcl::PointIndices::Ptr inliers (new pcl::PointIndices);  // indices in seg_pointcloud
            pcl::PointIndices ori_inliers;  // indices in ori_input_pointcloud
            pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
            seg.setInputCloud (single_pc);
            seg.segment (*inliers, *coefficients);
            stop=clock();
            cout << "RANSAC cost time : " << double(stop-start)/CLOCKS_PER_SEC << endl;
            if (inliers->indices.size () == 0)
            {
                std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            cout << "plane size : " << inliers->indices.size() << endl;
            //store point indices , cofficient
            extract_from_indices(ori_inliers, *inliers, seg_indices[j]);
            seg_plane_indices.push_back(ori_inliers);
            seg_plane_ModelCoefficients.push_back(*coefficients);
            
            // Remove the planar inliers, extract the rest 
            pcl::PointCloud<pcl::PointXYZ>::Ptr remain_pc (new pcl::PointCloud<pcl::PointXYZ>);
            Extract_Indices (single_pc, remain_pc, inliers, true);
            *single_pc = *remain_pc;
        }
        cout << "For RG cluster 0-" << j << " : plane number is : " << seg_plane_indices.size() << endl;
        //cout << "For RG cluster 0-" << j << " : plane number is : " << seg_plane_ModelCoefficients.size() << endl;
    }


/*------------------ extract all plane points -------------------*/
/*---------------------------------------------------------------*/
    vector<PointCloudXYZ::Ptr> plane_ptr_group;
    for(int n = 0; n < seg_plane_indices.size(); n++)
    {
        PointCloudXYZ::Ptr plane_ptr(new PointCloudXYZ);
        pcl::PointIndices input_idx = seg_plane_indices[n];
        extract_from_indices(plane_ptr, input_idx, pointcloud_ptr);
        plane_ptr_group.push_back(plane_ptr);
        cout << "plane size : " << plane_ptr->points.size() << endl;
        visualizer_window(plane_ptr, "Plane sequence obtained from pointcloud segmentation", true);
    }
    cout << "Finish extracting plane points from pointcloud_ptr" << endl;


/*------------------------------ extract contour from planes ------------------------------*/
/*-----------------------------------------------------------------------------------------*/
    // prepare parameters for concave hull


    vector< vector<pcl::PointXYZ> > plane_vertices_group;
    for(int i=0; i<plane_ptr_group.size(); i++)
    {
        // make plane ptr , coefficient ptr
        ModelCoefficients::Ptr single_coefficient_ptr = boost::make_shared< ModelCoefficients >(seg_plane_ModelCoefficients[i]);

        // Project the model inliers
        PointCloudXYZ::Ptr single_plane_project (new PointCloudXYZ);
        Project_Inliers(plane_ptr_group[i], single_coefficient_ptr, single_plane_project);
        visualizer_window( single_plane_project, "06 cloud cluster", false);

        // get plane contours
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        start=clock();
        Concave_Hull(single_plane_project, cloud_hull, alpha);
        stop=clock();
        cout << "get contour cost time : " << double(stop-start)/CLOCKS_PER_SEC << endl;
//      writer.write("/home/csl/catkin_ws/src/octree1/data/pc_contour1.pcd",*cloud_hull);
        visualizer_window( cloud_hull, "08 cloud hull after crop hull filtering", true);

        // get plane Vertices and store for later usage
        std::vector<pcl::PointXYZ> contour_points;
        start=clock();
        getPlaneApproxVertices(cloud_hull, single_coefficient_ptr, contour_points);
        stop=clock();
        cout << "Douglas-Peucker cost time : " << double(stop-start)/CLOCKS_PER_SEC << endl;
        plane_vertices_group.push_back(contour_points);

        // output plane modelCoefficients
        cout << "plane " << i << " coefficient : " << endl << *single_coefficient_ptr << endl;
    }
    
    
/*------------------ output pointcloud xyz as txt ----------------*/
/*----------------------------------------------------------------*/
    ofstream vertex_fout("plane_contour_xyz", ios::out);
    for(int i = 0; i < plane_vertices_group.size(); i++)
    {
        vertex_fout << i+1 << " plane : " << endl;
        for(int j=0; j<plane_vertices_group[i].size(); j++)
        {
            vertex_fout << plane_vertices_group[i][j].x << ',' << plane_vertices_group[i][j].y << ',' << plane_vertices_group[i][j].z << endl;
        }
        pcl::PointXYZ centerPoint;
        centerPoint = getCenterPoint(plane_vertices_group[i]);
        vertex_fout << "center point : " << centerPoint.x << "," << centerPoint.y << "," << centerPoint.z << endl;
        vertex_fout <<  " ****************** " << endl;
    }
    vertex_fout.close();
    
/*---------------------------- visualization ----------------------------*/
/*-----------------------------------------------------------------------*/
    // color table
    Eigen::MatrixXd colorMat(8,3); // red green blue yellow purple orange while darkgreen
    colorMat << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0,
                1.0, 1.0, 0.0,
                0.627, 0.125, 0.941,
                1.0, 0.647, 0.0,
                1.0, 1.0, 1.0,
                0.0, 0.392, 0.0;
                
    map<int, string> colorList;
    colorList[1] = "red";
    colorList[2] = "green";
    colorList[3] = "blue";
    colorList[4] = "yellow";
    colorList[5] = "purple";
    colorList[6] = "orange";
    colorList[7] = "while";
    colorList[8] = "darkgreen";
    //map[2] = "green";
    map<int, string>::iterator iter;
    
    for(iter = colorList.begin(); iter != colorList.end(); iter++){
        cout << iter->first << " " << iter->second << endl;
    }
                

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_1 (new visualization::PCLVisualizer ("3D Viewer"));
    viewer_1->setBackgroundColor (0, 0, 0);
    viewer_1->addCoordinateSystem (0.5);
    //viewer_1->initCameraParameters ();
    std::stringstream num_str;
    int line_num =0;
    
//     // add map original
//     pcl::PointXYZ p1;
//     p1.x = 0;    p1.y = 0; p1.z = 0;
//     viewer_1->addSphere (p1, 0.2, 1, 0, 0, "sphere1");

//     // TEST: add a plane
//     viewer_1->addPlane (seg_plane_ModelCoefficients[0], "plane0");


    for(int i=0; i<plane_vertices_group.size(); i++)
    {
        for(int j=0; j<plane_vertices_group[i].size()-1; j++)
        {
            num_str.clear();
            num_str<<"edge "<<line_num;
            
            viewer_1->addLine (plane_vertices_group[i][j], plane_vertices_group[i][j+1], colorMat(i,0), colorMat(i,1), colorMat(i,2), "edge_"+num_str.str());
            if(j==plane_vertices_group[i].size()-2)
                viewer_1->addLine (plane_vertices_group[i][j+1], plane_vertices_group[i][0], colorMat(i,0), colorMat(i,1), colorMat(i,2), "edge_closure"+num_str.str());
            line_num++;
        }
        
        
    }

    while(!viewer_1->wasStopped())
    {
        viewer_1->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }


/*------------------ trim plane pointcloud ------------------*/
/*-----------------------------------------------------------*/
    PointCloudXYZ::Ptr remain_pc(new PointCloudXYZ);
    PointCloudXYZ::Ptr input_pointcloud = boost::make_shared<PointCloudXYZ>(*pointcloud_ptr);
    for(int n = 0; n < seg_plane_indices.size(); n++)
    {
        PointIndices::Ptr single_palne_ptr = boost::make_shared< PointIndices >(seg_plane_indices[n]);
        Extract_Indices (input_pointcloud, remain_pc, single_palne_ptr, true, true);
        cout << "remain pc size : " << remain_pc->points.size() << endl;
        *input_pointcloud = *remain_pc;
        //visualizer_window( input_pointcloud, "remove all plane" );
        //cout << "after removing a cluster of plane pointcloud, it still has : " << input_pointcloud->points.size() << endl;
    }


  return (0);
}
