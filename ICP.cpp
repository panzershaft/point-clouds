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


ICP::ICP() {}

ICP::~ICP() {}

void ICP::remove_nan(ICP::PointCloudT::Ptr cloud_in) {
  vector < int > index1;
  pcl::removeNaNFromPointCloud( * cloud_in, * cloud_in, index1);
}

int ICP::ICPalgorithm(ICP::PointCloudT::Ptr cloud_icp,
  ICP::PointCloudT::Ptr cloud_tr) {

  ICP::icp icp;
  icp.setInputSource(cloud_icp);
  icp.setInputTarget(cloud_tr);

  ICP::PointCloudT Final;
  icp.align(Final);
  pcl::io::savePCDFileASCII ("icp_pcd.pcd", Final);

  cout << "\nhas converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << endl;
  cout << icp.getFinalTransformation() << endl;
  return (0);

}

void ICP::go_voxel(ICP::PointCloudT::Ptr cloud_a,
        ICP::PointCloudT::Ptr cloud_filter){

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud_a);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filter);
}

void ICP::colour_time(ICP::PointCloudT::Ptr cloud_filtered_two,
        ICP::PointCloudT::Ptr cloud_icp){
    
    printf("\nPoint cloud colors :  white  = original point cloud\n"
        "                        red  = transformed point cloud\n");
    ICP::viz viewer("Matrix transformation example");

    ICP::CustomColour source_cloud_color_handler(cloud_filtered_two, 255, 255, 255);
    viewer.addPointCloud(cloud_filtered_two, source_cloud_color_handler, "target cloud");

    ICP::CustomColour transformed_cloud_color_handler(cloud_icp, 230, 20, 20); // Red
    viewer.addPointCloud(cloud_icp, transformed_cloud_color_handler, "aligned cloud");

    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); 
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned cloud");
    
    // Display the visualiser until 'q' key is pressed
    while (!viewer.wasStopped()) { viewer.spinOnce(); }

}

int ICP::file_loader() {
  ICP::PointCloudT::Ptr cloud_one(new ICP::PointCloudT);
  ICP::PointCloudT::Ptr cloud_two(new ICP::PointCloudT);
  ICP::PointCloudT::Ptr cloud_filtered_one (new ICP::PointCloudT);
  ICP::PointCloudT::Ptr cloud_filtered_two (new ICP::PointCloudT);
  ICP::PointCloudT::Ptr cloud_icp (new ICP::PointCloudT);

  pcl::PCDReader reader;
  
  reader.read ("frame1.pcd", *cloud_one); 
  ICP::remove_nan(cloud_one);
  ICP::go_voxel(cloud_one, cloud_filtered_one);

  reader.read ("frame10.pcd", *cloud_two);
  ICP::remove_nan(cloud_two);
  ICP::go_voxel(cloud_two, cloud_filtered_two);

  ICP::ICPalgorithm(cloud_filtered_one, cloud_filtered_two);

  reader.read ("icp_pcd.pcd", *cloud_icp);

  ICP::colour_time(cloud_filtered_two, cloud_icp);
  
}