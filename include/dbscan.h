#ifndef DBSCAN_H
#define DBSCAN_H

#include "my_point_cloud.h"


int dbscan(PointCloudPtr input, std::vector<int> &labels, double eps, int min);

#endif /*DBSCAN_H*/
