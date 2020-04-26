/*    
 *     
 * Created by Soham 26/04/2020
 *   
 */
#include"ICP.hpp"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>


IcpTime::IcpTime() {}

IcpTime::~IcpTime() {}

void IcpTime::remove_nan(IcpTime::PointCloudT::Ptr cloud_in) {
  std::vector < int > index1;
  pcl::removeNaNFromPointCloud( * cloud_in, * cloud_in, index1);
}

int IcpTime::ICPalgorithm(IcpTime::PointCloudT::Ptr cloud_icp,
  IcpTime::PointCloudT::Ptr cloud_tr) {

  pcl::IterativeClosestPoint < pcl::PointXYZ, pcl::PointXYZ > icp;
  
  icp.setInputSource(cloud_icp);
  icp.setInputTarget(cloud_tr);

  pcl::PointCloud < pcl::PointXYZ > Final;
  icp.align(Final);

  std::cout << "\nhas converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  return (0);

}

void IcpTime::go_voxel(IcpTime::PointCloudT::Ptr cloud_a,
        IcpTime::PointCloudT::Ptr cloud_filter){
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud_a);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filter);
}

int IcpTime::file_loader() {
  IcpTime::PointCloudT::Ptr cloud_one(new IcpTime::PointCloudT);
  IcpTime::PointCloudT::Ptr cloud_two(new IcpTime::PointCloudT);
  IcpTime::PointCloudT::Ptr cloud_model(new IcpTime::PointCloudT);
  IcpTime::PointCloudT::Ptr cloud_filtered_one (new IcpTime::PointCloudT);
  IcpTime::PointCloudT::Ptr cloud_filtered_two (new IcpTime::PointCloudT);

  pcl::PCDReader reader;
  
  reader.read ("frame1.pcd", *cloud_one); 
  IcpTime::remove_nan(cloud_one);
  IcpTime::go_voxel(cloud_one, cloud_filtered_one);

  reader.read ("frame10.pcd", *cloud_two);
  IcpTime::remove_nan(cloud_two);
  IcpTime::go_voxel(cloud_two, cloud_filtered_two);

  IcpTime::ICPalgorithm(cloud_filtered_one, cloud_filtered_two);
}