/*    
 *     
 * Created by Soham 27/04/2020
 *   
 */

#ifndef INTERACTIVEICP_HPP_INCLUDED
#define INTERACTIVEICP_INCLUDED
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
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;
typedef pcl::visualization::PCLVisualizer viz;
typedef pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > CustomColour;
typedef pcl::IterativeClosestPoint < pcl::PointXYZ, pcl::PointXYZ > icp;

class  InteractiveICP{

private:        
    PointCloudT::Ptr cloud_one;
    PointCloudT::Ptr cloud_two;
    PointCloudT::Ptr cloud_in;
    PointCloudT::Ptr cloud_tr;
    PointCloudT::Ptr cloud_icp;
    
    
public:
    InteractiveICP();
    ~InteractiveICP();	 
    int file_loader();
    void remove_nan(PointCloudT::Ptr cloud_in);
    void go_voxel(PointCloudT::Ptr cloud_a, PointCloudT::Ptr cloud_b);
    void colour_time(PointCloudT::Ptr cloud_a, PointCloudT::Ptr cloud_b, PointCloudT::Ptr cloud_c);
    
};

#endif