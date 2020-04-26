/*    
 *     
 * Created by Soham 26/04/2020
 *   
 */

#ifndef ICPTIME_HPP_INCLUDED
#define ICPTIME_INCLUDED
#include <iostream>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/registration/icp.h>
using namespace std;

class  IcpTime{
public:
    typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
    IcpTime();
    ~IcpTime();	 
    int file_loader();
    void remove_nan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
    int ICPalgorithm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tr);
    
};

#endif