#pragma once

#include <opencv2/opencv.hpp> 
#include <vector>
#include <string>
#include <iostream>

#include <pcl/point_types.h>
#include "Parameter_Reader.h"

using namespace std;
using namespace cv;

ParameterReader pr;

void point_xyz2uv(pcl::PointCloud<pcl::PointXYZ>::Ptr plane_vetexes, vector<cv::Point> &uv_points, int num_points)
{
    // get the camera parameters
    float camera_cx = atof( pr.getData( "camera_cx" ).c_str());
    float camera_cy = atof( pr.getData( "camera_cy" ).c_str());
    float camera_fx = atof( pr.getData( "camera_fx" ).c_str());
    float camera_fy = atof( pr.getData( "camera_fy" ).c_str());
    cout << "camera_cx : " << camera_cx << endl;
    cout << "camera_fx : " << camera_fx << endl;

    // project the plane vetexes from xyz to uv 
    //int num_points = plane_vetexes->points.size();
    //CvPoint uv_points[num_points];
    //uv_pointer = uv_points;
    for(int i = 0; i < num_points; i++)
    {
        pcl::PointXYZ point = plane_vetexes->points[i];
        int u = point.x * camera_fx / point.z + camera_cx;
        int v = point.y * camera_fy / point.z + camera_cy;
        cv::Point p1(u,v);
        uv_points.push_back(p1);
    }

}


void draw_polygon(vector<cv::Point> contour, int num)
{
    cv::Mat img_color = imread("/home/gordon/feelEnvironment/data/color.png", 1);
    cout << "number of plane vetexes : " << num << endl;

    //cvPolyLine( color, &_arr, &num, 1, 1, CV_RGB(0,255,0), 2, CV_AA, 0 );
    vector<vector<cv::Point>> contours;
    contours.push_back(contour);
    vector< cv::Vec4i > hierarchy;
    cv::drawContours( img_color, contours, -1, cv::Scalar(0,100,255), 2, 8, hierarchy, 1 );
    
    float plane_area = contourArea(contour, false);
    cout << "plane area is : " << plane_area << endl;

    cv::imshow("draw hull", img_color);
    cv::waitKey();
}


void normal_groundtruth()
{
    // 读取图像，从中提取出角点，然后对角点进行亚像素精确化
    Size image_size;      /* 图像的尺寸 */
    Size board_size = Size(11, 8);             /* 标定板上每行、列的角点数 */
    vector<Point2f> image_points_buf;         /* 缓存每幅图像上检测到的角点 */
//    vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */

    Mat imageInput = imread("/home/gordon/feelEnvironment/data/color.png");
    image_size.width = imageInput.cols;
    image_size.height = imageInput.rows;

    /* 提取角点 */
    if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
    {           
        cout << "can not find chessboard corners!\n";  // 找不到角点
        exit(1);
    } 
    else 
    {
        Mat view_gray;
        cvtColor(imageInput, view_gray, CV_RGB2GRAY);  // 转灰度图

        /* 亚像素精确化 */
        // image_points_buf 初始的角点坐标向量，同时作为亚像素坐标位置的输出
        // Size(5,5) 搜索窗口大小
        // （-1，-1）表示没有死区
        // TermCriteria 角点的迭代过程的终止条件, 可以为迭代次数和角点精度两者的组合
        cornerSubPix(view_gray, image_points_buf, Size(5,5), Size(-1,-1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

//        image_points_seq.push_back(image_points_buf);  // 保存亚像素角点

        /* 在图像上显示角点位置 */
        drawChessboardCorners(view_gray, board_size, image_points_buf, false); // 用于在图片中标记角点

        imshow("Camera Calibration", view_gray);       // 显示图片

        waitKey(); //暂停0.5S      
    }

    int CornerNum = board_size.width * board_size.height;  // 每张图片上总的角点数

    cout << "finish searching for chessboard corners" << endl;

    /*棋盘三维信息*/
    Size square_size = Size(30, 30);         /* 实际测量得到的标定板上每个棋盘格的大小 */
    vector<Point3f> object_points;   /* 保存标定板上角点的三维坐标 */

    /*内外参数*/
    //float factory_in[3][3] = {{615.899, 0, 321.798},{0, 616.468, 239.607},{0, 0, 1}};   /*realsense 厂家内参*/
    //float calib_in[3][3] = {{613.2318106271917, 0, 322.6972952154207},{0, 612.7423040195675, 236.8980898837235},{0, 0, 1}};   /*realsense 标定内参*/
    //float factory_in[3][3] = {{525, 0, 319.5},{0, 525.0, 239.5},{0, 0, 1}};   /*xtion 厂家内参*/
    float calib_in[3][3] = {{536.7864099962015, 0, 322.1897304865881},{0, 536.7545903015535, 237.1985000831648},{0, 0, 1}};   /*xtion 标定内参*/


    Mat cameraMatrix = Mat(3, 3, CV_32FC1, calib_in);  /* 摄像机内参数矩阵 */
    vector<int> point_counts;   // 每幅图像中角点的数量
    //float calib_dist[5] = {0.009667384003958859, 0.8474904796206151, 0.00105995694879978, 0.00113169560199054, -3.200938696914244};   /*realsense 畸变系数*/
    float calib_dist[5] = {0.03354841994589133, -0.0204169693295175, -0.0004610056977813335, 0.0001788220783987586, -0.2779805557257407};     /*xtion 畸变系数*/
    Mat distCoeffs=Mat(1, 5, CV_32FC1, calib_dist);       /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    Mat tvecsMat;      /* 每幅图像的平移向量 */
    Mat rvecsMat;      /* 每幅图像的旋转向量 */

    /* 初始化标定板上角点的三维坐标 */
    for (int i=0; i<board_size.height; i++) 
    {
        for (int j=0; j<board_size.width; j++) 
        {
            Point3f realPoint;

            /* 假设标定板放在世界坐标系中z=0的平面上 */
            realPoint.x = i * square_size.width;
            realPoint.y = j * square_size.height;
            realPoint.z = 0;
            object_points.push_back(realPoint);
        }
    }

    /* 计算相机外参 */
    // 0 is the flag.Method for solving a PnP problem
    if( 0 == solvePnP(object_points, image_points_buf, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, false, 0))
    {
        cout << "fail to run solvePnP!" << endl;
        return ;
    }

    // -------------------对计算结果进行评价------------------------------

    ofstream fout("/home/gordon/feelEnvironment/data/solvePnP_result.txt");  /* 保存标定结果的文件 */
    double err = 0.0;               /* 每幅图像的平均误差 */
    vector<Point2f> image_points2;  /* 保存重新计算得到的投影点 */
    fout<<"每幅图像的标定误差：\n";


    vector<Point3f> tempPointSet = object_points;

    /* 通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点 */
    projectPoints(object_points, rvecsMat, tvecsMat, cameraMatrix, distCoeffs, image_points2);

    /* 计算新的投影点和旧的投影点之间的误差*/
    vector<Point2f> tempImagePoint = image_points_buf;
    Mat tempImagePointMat = Mat(1, tempImagePoint.size(), CV_32FC2);
    Mat image_points2Mat = Mat(1, image_points2.size(), CV_32FC2);

    for (int j = 0 ; j < tempImagePoint.size(); j++)
    {
       image_points2Mat.at<Vec2f>(0,j) = Vec2f(image_points2[j].x, image_points2[j].y);
       tempImagePointMat.at<Vec2f>(0,j) = Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
    }
    err = norm(image_points2Mat, tempImagePointMat, NORM_L2); 
    fout << "图像的平均误差：" << err<< "像素" << endl;   

    //-------------------------评价完成---------------------------------------------

    // 打印棋盘格法向量
    Mat rotation_matrix = Mat(3,3,CV_32FC1, Scalar::all(0));  /* 保存每幅图像的旋转矩阵 */

    fout << "图像的旋转向量：" << endl;   
    fout << tvecsMat << endl;

    /* 将旋转向量转换为相对应的旋转矩阵 */   
    Rodrigues(tvecsMat, rotation_matrix);   
    fout << "图像的旋转矩阵：" << endl;   
    fout << rotation_matrix << endl;  
    fout.close();

}
