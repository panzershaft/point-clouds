/*    
 *     
 * Created by Soham 26/04/2020
 *   
 */
#include"InteractiveICP.hpp"
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

pcl::VoxelGrid<pcl::PointXYZ> vg;     
pcl::PCDReader reader;

bool next_iteration = false;
Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

InteractiveICP::InteractiveICP()  :cloud_one(new PointCloudT()), 
                                  cloud_two(new PointCloudT()),
                                  cloud_in(new PointCloudT()),
                                  cloud_tr(new PointCloudT()),
                                  cloud_icp(new PointCloudT()),
                                  cloud_model(new PointCloudT()){}
InteractiveICP::~InteractiveICP() {}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *nothing)
{
  if (event.getKeySym() == "space" && event.keyDown())
  {
    next_iteration = true;
  }
}

void InteractiveICP::remove_nan(PointCloudT::Ptr cloud_in) 
{
  vector < int > index1;
  pcl::removeNaNFromPointCloud( * cloud_in, * cloud_in, index1);
}

void InteractiveICP::go_voxel(PointCloudT::Ptr cloud_a, PointCloudT::Ptr cloud_filter)
{
    
    vg.setInputCloud (cloud_a);
    vg.setLeafSize (0.01f, 0.01f, 0.01f);
    vg.filter (*cloud_filter);
}

void InteractiveICP::down_sampler(PointCloudT::Ptr cloud_a, PointCloudT::Ptr cloud_b)
{
  InteractiveICP::remove_nan(cloud_a);
  InteractiveICP::go_voxel(cloud_a, cloud_b);
}

int InteractiveICP::file_loader() 
{
  icp icp;
  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();
  viz viewer("ICP example");
  reader.read ("/home/soham/wow/src/my_pcl_tutorial/Pure_Filter/frame1.pcd", *cloud_one); 
  InteractiveICP::down_sampler(cloud_one, cloud_in);

  reader.read ("/home/soham/wow/src/my_pcl_tutorial/Pure_Filter/frame10.pcd", *cloud_two);
  InteractiveICP::down_sampler(cloud_two, cloud_tr);
  *cloud_icp = *cloud_in;
  
  
  CustomColour source_cloud_color_handler(cloud_in, 255, 255, 255);
  viewer.removePointCloud("source_cloud_color_handler");
  viewer.addPointCloud(cloud_in, source_cloud_color_handler, "source_cloud_color_handler");

  CustomColour target_cloud_color_handler(cloud_tr, 20, 180, 20);
  viewer.removePointCloud("target_cloud_color_handler");
  viewer.addPointCloud(cloud_tr, target_cloud_color_handler, "target_cloud_color_handler");

  CustomColour transformed_cloud_color_handler(cloud_icp, 230, 20, 20); // Red
  viewer.removePointCloud("transformed_cloud_color_handler");
  viewer.addPointCloud(cloud_icp, transformed_cloud_color_handler, "transformed_cloud_color_handler");

  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); 
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_cloud_color_handler");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_color_handler");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud_color_handler");
  

  viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *)NULL);
  if(viewer.wasStopped()== true){
          viewer.spinOnce(); 
      }

  while (!viewer.wasStopped())
      {
        viewer.spinOnce();

        if (next_iteration)
        {
          icp.setInputSource(cloud_icp);
          icp.setInputTarget(cloud_tr);
          icp.align(*cloud_icp);

          if (icp.hasConverged())
          {
            printf("\nPoint cloud colors :  white  = source point cloud\n"
                  "                       green  = target point cloud\n"
                  "                       red  = transformed point cloud\n");
            cout << "\nhas converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
            transformation_matrix *=  icp.getFinalTransformation().cast<double>();
            pcl::transformPointCloud (*cloud_in, *cloud_model, transformation_matrix);
            cout << icp.getFinalTransformation() << endl;
            std::cout << "\nICP has converged, score is " <<  icp.getFitnessScore() << std::endl;
            viewer.updatePointCloud(cloud_in, source_cloud_color_handler, "source_cloud_color_handler");
            viewer.updatePointCloud(cloud_tr, target_cloud_color_handler, "target_cloud_color_handler");
            viewer.updatePointCloud(cloud_icp, transformed_cloud_color_handler, "transformed_cloud_color_handler");
          }
          else
          {
            PCL_ERROR("\nICP has not converged.\n");
            return (-1);
          }
        }
        next_iteration = false;
    }
}