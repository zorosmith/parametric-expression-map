#include "dbscan.h"
#include <math.h>
#include <algorithm>

static const inline double distance(double x1, double y1, double z1, double x2, double y2, double z2)
{
    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;

    return sqrt(dx * dx + dy * dy + dz * dz);
}

const inline int region_query(PointCloudPtr input, int p, std::vector<int> &output, double eps)
{
    for(int i = 0; i < (int)input->points.size(); i++){

        if(distance(input->points[i].x, input->points[i].y, input->points[i].z, input->points[p].x, input->points[p].y, input->points[p].z) < eps){
            output.push_back(i);
        }
    }

    return output.size();
}

bool expand_cluster(PointCloudPtr input, int p, std::vector<int> &output, int cluster, double eps, int min)
{
    std::vector<int> seeds;

    if(region_query(input, p, seeds, eps) < min){

        //this point is noise
        output[p] = -1;
        return false;

    }else{

        //set cluster id
        for(int i = 0; i < (int)seeds.size(); i++){
            output[seeds[i]] = cluster;
        }

//        for(std::vector<int>::iterator it=seeds.begin();it!=seeds.end();it++)
//            std::cout<<*it<<" ";
//        std::cout << std::endl;

        //delete paint from seeds. remove the point input[p]
        seeds.erase(std::remove(seeds.begin(), seeds.end(), p), seeds.end());

//       for(std::vector<int>::iterator it=seeds.begin();it!=seeds.end();it++)
//            std::cout<<*it<<" ";
//        std::cout << std::endl;

        //seed -> empty
        while((int)seeds.size() > 0){

            int cp = seeds.front();
            std::vector<int> result;

            if(region_query(input, cp, result, eps) >= min){

                for(int i = 0; i < (int)result.size(); i++){

                    int rp = result[i];

                    //this paint is noise or unmarked point
                    if(output[rp] < 1){

                        //unmarked point
                        if(!output[rp]){
                            seeds.push_back(rp);
                        }

                        //set cluster id
                        output[rp] = cluster;
                    }
                }
            }

            //delete point from seeds
            seeds.erase(std::remove(seeds.begin(), seeds.end(), cp), seeds.end());
        }
    }

    return true;
}

int dbscan(PointCloudPtr input, std::vector<int> &labels, double eps, int min)
{
    int size = input->points.size();
    int cluster = 1;

    std::vector<int> state(size);

    for(int i = 0; i < size; i++){

        if(!state[i]){

            if(expand_cluster(input, i, state, cluster, eps, min)){
                cluster++;
            }
        }
    }

    labels = state;

    return cluster - 1;
}
