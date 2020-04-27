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


ICP::ICP()  :cloud_one(new PointCloudT()), 
             cloud_two(new PointCloudT()),
             cloud_in(new PointCloudT()),
             cloud_tr(new PointCloudT()),
             cloud_icp(new PointCloudT())
             
{}
ICP::~ICP() {}

void ICP::remove_nan(PointCloudT::Ptr cloud_in) {
  vector < int > index1;
  pcl::removeNaNFromPointCloud( * cloud_in, * cloud_in, index1);
}


int ICP::run_ICP_algorithm() {

  icp icp;
  icp.setInputSource(cloud_icp);
  icp.setInputTarget(cloud_tr);

  // PointCloudT Final = *cloud_in;
  icp.align(*cloud_icp);
  // pcl::io::savePCDFileASCII ("icp_pcd.pcd", Final);

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

void ICP::colour_time(){
    
    printf("\nPoint cloud colors :  white  = source point cloud\n"
        "                        green  = target point cloud\n"
        "                        red  = transformed point cloud\n");
    viz viewer("ICP example");

    CustomColour source_cloud_color_handler(cloud_in, 255, 255, 255);
    viewer.addPointCloud(cloud_in, source_cloud_color_handler, "source cloud");

    CustomColour target_cloud_color_handler(cloud_tr, 20, 180, 20);
    viewer.addPointCloud(cloud_tr, target_cloud_color_handler, "target cloud");

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
  pcl::PCDReader reader;

  reader.read ("frame1.pcd", *cloud_one); 
  // pcl::io::loadPCDFile("/home/soham/wow/src/my_pcl_tutorial/k2b/frame0.pcd", *cloud_one);
  ICP::remove_nan(cloud_one);
  ICP::go_voxel(cloud_one, cloud_in);

  reader.read ("frame10.pcd", *cloud_two);
  // pcl::io::loadPCDFile("/home/soham/wow/src/my_pcl_tutorial/k2b/frame8.pcd", *cloud_two);
  ICP::remove_nan(cloud_two);
  ICP::go_voxel(cloud_two, cloud_tr);
  
  *cloud_icp = *cloud_in;
  ICP::run_ICP_algorithm();
  // reader.read ("icp_pcd.pcd", *cloud_icp);

  ICP::colour_time();
  
}