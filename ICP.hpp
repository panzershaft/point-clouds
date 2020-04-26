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
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

using namespace std;

class  IcpTime{
public:
    typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
    typedef pcl::VoxelGrid<pcl::PointXYZ> vg;
    IcpTime();
    ~IcpTime();	 
    int file_loader();
    void remove_nan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
    int ICPalgorithm(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_icp,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tr);
    void go_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_a, 
    IcpTime::PointCloudT::Ptr cloud_b);
    
};

#endif