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
#include <vector>
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
    PointCloudT::Ptr cloud_in;
    PointCloudT::Ptr cloud_tr;
    PointCloudT::Ptr cloud_icp;
    PointCloudT::Ptr cloud_static;
    PointCloudT::Ptr cloud_model;
    int step_size;
    pcl::VoxelGrid < pcl::PointXYZ > vg;
    pcl::PCDReader reader;
    stringstream sstream;
    string input_path {"/home/soham/lmr_project/3d_files/frame"};
    std::vector<Eigen::Matrix4d> poses;
    Eigen::Matrix4d final_pose;
    Eigen::Matrix4d total_matrix;
    Eigen::Matrix4d transformation_matrix;
    Eigen::Matrix4d current_matrix;
    
public:
    InteractiveICP();
    ~InteractiveICP();	 
    int Runner();
    void remove_nan(PointCloudT::Ptr cloud_in);
    void go_voxel(PointCloudT::Ptr cloud_a);
    void down_sampler(PointCloudT::Ptr cloud_a);
    int fileLoader(int i, int step_size, PointCloudT::Ptr cloud_a, PointCloudT::Ptr cloud_b, PointCloudT::Ptr cloud_c);
    int PCL_file_not_found(){ PCL_ERROR("Couldn't read the file! \n"); // In case of file not being read
    return (-1);}
    int ICP_not_converged(){ PCL_ERROR("\nICP has not converged.\n");
    return (-1);}
    Eigen::Matrix4d compute_compound_transforamtion(const vector<Eigen::Matrix4d> &poses);
};

#endif