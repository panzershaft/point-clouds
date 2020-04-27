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

void ICP::remove_nan(PointCloudT::Ptr cloud_in) {
  vector < int > index1;
  pcl::removeNaNFromPointCloud( * cloud_in, * cloud_in, index1);
}

int ICP::run_ICP_algorithm(PointCloudT::Ptr cloud_filtered_one,
  PointCloudT::Ptr cloud_filtered_two) {

  icp icp;
  icp.setInputSource(cloud_filtered_one);
  icp.setInputTarget(cloud_filtered_two);

  PointCloudT Final;
  icp.align(Final);
  pcl::io::savePCDFileASCII ("icp_pcd.pcd", Final);

  cout << "\nhas converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << endl;
  cout << icp.getFinalTransformation() << endl;
  return (0);

}

void ICP::go_voxel(PointCloudT::Ptr cloud_a,
        PointCloudT::Ptr cloud_filter){

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud_a);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filter);
}

void ICP::colour_time(PointCloudT::Ptr cloud_filtered_one,
        PointCloudT::Ptr cloud_filtered_two,
        PointCloudT::Ptr cloud_icp){
    
    printf("\nPoint cloud colors :  white  = source point cloud\n"
        "                        green  = target point cloud\n"
        "                        red  = transformed point cloud\n");
    viz viewer("ICP example");

    CustomColour source_cloud_color_handler(cloud_filtered_one, 255, 255, 255);
    viewer.addPointCloud(cloud_filtered_one, source_cloud_color_handler, "source cloud");

    CustomColour target_cloud_color_handler(cloud_filtered_two, 20, 180, 20);
    viewer.addPointCloud(cloud_filtered_two, target_cloud_color_handler, "target cloud");

    CustomColour transformed_cloud_color_handler(cloud_icp, 230, 20, 20); // Red
    viewer.addPointCloud(cloud_icp, transformed_cloud_color_handler, "aligned cloud");

    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); 
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligned cloud");
    
    // Display the visualiser until 'q' key is pressed
    while (!viewer.wasStopped()) { viewer.spinOnce(); }

}

int ICP::file_loader() {
  ICP::cloud_one = PointCloudT::Ptr(new PointCloudT);
  ICP::cloud_two = PointCloudT::Ptr(new PointCloudT);
  ICP::cloud_filtered_one = PointCloudT::Ptr(new PointCloudT);
  ICP::cloud_filtered_two = PointCloudT::Ptr(new PointCloudT);
  ICP::cloud_icp = PointCloudT::Ptr(new PointCloudT);

  pcl::PCDReader reader;
  
  reader.read ("frame1.pcd", *cloud_one); 
  // pcl::io::loadPCDFile("/home/soham/wow/src/my_pcl_tutorial/k2b/frame0.pcd", *cloud_one);
  ICP::remove_nan(cloud_one);
  ICP::go_voxel(cloud_one, cloud_filtered_one);

  reader.read ("frame10.pcd", *cloud_two);
  // pcl::io::loadPCDFile("/home/soham/wow/src/my_pcl_tutorial/k2b/frame8.pcd", *cloud_two);
  ICP::remove_nan(cloud_two);
  ICP::go_voxel(cloud_two, cloud_filtered_two);

  ICP::run_ICP_algorithm(cloud_filtered_one, cloud_filtered_two);

  reader.read ("icp_pcd.pcd", *cloud_icp);

  ICP::colour_time(cloud_filtered_one, cloud_filtered_two, cloud_icp);
  
}