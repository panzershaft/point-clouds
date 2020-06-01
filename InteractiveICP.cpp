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

bool next_iteration = false;

InteractiveICP::InteractiveICP():
  cloud_in(new PointCloudT()),
  cloud_tr(new PointCloudT()),
  cloud_icp(new PointCloudT()),
  cloud_static(new PointCloudT()),
  cloud_model(new PointCloudT()) {}

InteractiveICP::~InteractiveICP() {}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent & event, void * nothing) {
  if (event.getKeySym() == "space" && event.keyDown()) {
    next_iteration = true;
  }
}

void InteractiveICP::remove_nan(PointCloudT::Ptr cloud_in) {
  vector < int > file_index1;
  pcl::removeNaNFromPointCloud( * cloud_in, * cloud_in, file_index1);
}

void InteractiveICP::go_voxel(PointCloudT::Ptr cloud_voxeled) {
  vg.setInputCloud(cloud_voxeled);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter( * cloud_voxeled);
}

void InteractiveICP::down_sampler(PointCloudT::Ptr cloud_a) {
  InteractiveICP::remove_nan(cloud_a);
  InteractiveICP::go_voxel(cloud_a);
}


int InteractiveICP::fileLoader(int file_index, int step_size,
                               PointCloudT::Ptr cloud_in, 
                               PointCloudT::Ptr cloud_tr,
                               PointCloudT::Ptr cloud_icp) {
  sstream << input_path << "/frame" << file_index << ".pcd";
  string file_one = sstream.str();
  sstream.str(string());
  sstream << input_path << "/frame" << file_index + step_size << ".pcd";
  string file_two = sstream.str();
  sstream.str(string());

  cout << file_one << "\n" << file_two << "\n";

  if (reader.read(file_one, * cloud_in) == -1){ InteractiveICP::PCL_file_not_found(); }
  InteractiveICP::down_sampler(cloud_in);

  if (reader.read(file_two, * cloud_tr) == -1){ InteractiveICP::PCL_file_not_found(); }
  InteractiveICP::down_sampler(cloud_tr);

  if (file_index == 0){ *cloud_static = *cloud_in; }

  *cloud_icp = *cloud_in;
  return (0);
}

 Eigen::Matrix4d InteractiveICP::compute_compound_transforamtion(const vector<Eigen::Matrix4d> &poses){
  final_pose = Eigen::Matrix4d::Identity();
  
  for (const auto & p : poses)
    final_pose = p * final_pose;
  return final_pose;
  }

int InteractiveICP::Runner() {
  step_size = 10; //step_size depends on your data, 10 works better for my files
  icp icp;
  total_matrix = Eigen::Matrix4d::Identity ();
  viz viewer("Interactive ICP");

  for (int file_index = 0; file_index < 100; file_index+=step_size) {
    fileLoader(file_index, step_size, cloud_in, cloud_tr, cloud_icp);

    transformation_matrix = Eigen::Matrix4d::Identity();

    CustomColour icp_cloud_color_handler(cloud_icp, 80, 5, 431); // blue: cloud_icp
    viewer.removePointCloud("icp_cloud_color_handler");
    viewer.addPointCloud(cloud_icp, icp_cloud_color_handler, "icp_cloud_color_handler");


    CustomColour source_cloud_color_handler(cloud_static, 255, 255, 255); // white: First input cloud
    viewer.removePointCloud("source_cloud_color_handler");
    viewer.addPointCloud(cloud_static, source_cloud_color_handler, "source_cloud_color_handler");

    CustomColour target_cloud_color_handler(cloud_tr, 20, 180, 20); // green: target cloud
    viewer.removePointCloud("target_cloud_color_handler");
    viewer.addPointCloud(cloud_tr, target_cloud_color_handler, "target_cloud_color_handler");

    CustomColour transformed_cloud_color_handler(cloud_model, 230, 20, 20); // Red: compunded matrix
    viewer.removePointCloud("transformed_cloud_color_handler");
    viewer.addPointCloud(cloud_model, transformed_cloud_color_handler, "transformed_cloud_color_handler");

    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0);
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_cloud_color_handler");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_color_handler");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud_color_handler");
    
    viewer.registerKeyboardCallback( & keyboardEventOccurred, (void * ) NULL);
    
    if (viewer.wasStopped() == true) { viewer.spinOnce(); }

    total_matrix = compute_compound_transforamtion(poses);

    while (!viewer.wasStopped()) {
      viewer.spinOnce();

      if (next_iteration) {
        icp.setInputSource(cloud_icp);
        icp.setInputTarget(cloud_tr);
        icp.align( * cloud_icp);
        transformation_matrix = icp.getFinalTransformation().cast<double> () * transformation_matrix;
        if (icp.hasConverged()) {
          printf("\nPoint cloud colors :  white  = source point cloud\n"
            "                      green  = target point cloud\n"
            "                      blue: = transformed point cloud (cloud_icp)\n"
            "                      red  = transformed point cloud (compunded matrix of white)\n");
          cout << "\nhas converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << endl;
          
          current_matrix = transformation_matrix * total_matrix;

          pcl::transformPointCloud( * cloud_static, * cloud_model, current_matrix);
          cout << icp.getFinalTransformation() << endl;
          std::cout << "\nICP has converged, score is " << icp.getFitnessScore() << std::endl;
          
          viewer.updatePointCloud(cloud_icp, icp_cloud_color_handler, "icp_cloud_color_handler");
          viewer.updatePointCloud(cloud_static, source_cloud_color_handler, "source_cloud_color_handler");
          viewer.updatePointCloud(cloud_tr, target_cloud_color_handler, "target_cloud_color_handler");
          viewer.updatePointCloud(cloud_model, transformed_cloud_color_handler, "transformed_cloud_color_handler");
        } 
        else { InteractiveICP::ICP_not_converged(); }
      }
      next_iteration = false;
    }
    poses.push_back(transformation_matrix);
  }
  return (0);
}