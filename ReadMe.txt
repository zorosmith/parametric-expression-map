所有输入数据都调整了位置，运行程序很大可能会报错

记录一下这个混乱的project
1.核心程序： main_pipeline.cpp
该程序内的参数写在， ./parameters.txt
壁面结构化算法，提取出壁面的轮廓顶点和法向量，估计值

2.获取壁面轮廓的groundtruth
adjust_gt_intensity.cpp 用于选择rgb图的(光)强度阈值
draw_walledge.cpp 使用上述输出的(光)强度阈值提取壁面的轮廓，输出壁面轮廓的顶点坐标和面积，单位皆为像素。
运行方法：python3环境。命令行带参数运行,参数为图片

3.数据存储
真实值和估计值存储在 ./accuracy_data/result_analyse/record 文件中

4.评价壁面轮廓的精度
python_code 文件夹：
a，iou_calculate.py，
输入： 壁面轮廓的顶点坐标，groundtruth 和 结构化算法的估计值；壁面轮廓的面积，单位为像素；
修改建议： 改成由文本读取输入；
b，accuracy_hailun.py ，accuracy_cross_product.py 分别用海伦公式和划分三角形的方式计算多边形面积

5.相机标定
zhang_calibration.cpp
使用opencv2.4.9编译；
标定所需要的棋盘格图片存放在 ./build/calibration_pic。删减棋盘格图片时都要修改calibdata.txt

6.超体素聚类
supervoxel_lccp.cpp 需要pcl1.8编译

7.Bounding Box处理
moment_of_inertia_estimation.cpp 
