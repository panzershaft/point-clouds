
#include"matrix_transform.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>


matrix_transform::PointCloudT::Ptr cloud_one (new pcl::PointCloud<pcl::PointXYZ>);
matrix_transform::PointCloudT::Ptr cloud_two (new pcl::PointCloud<pcl::PointXYZ>);
matrix_transform::PointCloudT::Ptr cloud_source (new pcl::PointCloud<pcl::PointXYZ>);
matrix_transform::PointCloudT cloud_c;

void matrix_transform::remove_nan(matrix_transform::PointCloudT::Ptr cloud_in)
{
  //NaN removal
  std::vector<int> index1;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, index1);
}

void matrix_transform::lets_concat(matrix_transform::PointCloudT::Ptr cloud_a,
    matrix_transform::PointCloudT::Ptr cloud_b)
{  
      cloud_c  = *cloud_a;
      cloud_c += *cloud_b;
      std::cerr << "Cloud C: " << std::endl;
      pcl::io::savePCDFileASCII ("cloud_concatenate.pcd", cloud_c);
      std::cerr << "Saved " << cloud_c.points.size () << " data points to cloud_concatenate.pcd." << std::endl;
}

int matrix_transform::file_loader(int i)
{
      //stringstream ss;
      std::stringstream sstream;
      sstream << matrix_transform::input_path << "/frame" << i << ".pcd";
      string file1 = sstream.str();
      sstream.str(std::string());
      sstream << matrix_transform::input_path << "/frame" << i + 9 << ".pcd";
      string file2 = sstream.str();
      sstream.str(std::string());
      
      std::cout << "Loading: " << file1 << std::endl;
	  
      matrix_transform::remove_nan(cloud_one);
      if (pcl::io::loadPCDFile(file1, *cloud_one) == -1)
      {
      
        PCL_ERROR("Couldn't read first file! \n"); // In case of file not being read
        return (-1);
      
      }

      matrix_transform::remove_nan(cloud_two);
      std::cout << "Loading: " << file2 << std::endl;
      if (pcl::io::loadPCDFile(file2, *cloud_two) == -1)
      {
        PCL_ERROR("Couldn't read the second file! \n"); // In case of file not being read
        return (-1);
      }

      //now we add the two point clouds
      // matrix_transform::lets_concat(cloud_one, cloud_two);
	

      sstream.clear();
  	  file1.clear();
	    file2.clear();
}

int main()
{
	
	for(int i = 1; i < 2; i++)
	{
	    matrix_transform::file_loader(i);
     
	}
	
  if (pcl::io::loadPCDFile("/home/soham/lmr_project/build/cloud_a.pcd", *cloud_source) == -1)
    {
      PCL_ERROR("Couldn't read first file! \n"); // In case of file not being read
      return (-1);
    }
  float theta = M_PI/4; // The angle of rotation in radians 
  Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  transform_2.translation() << 2.5, 0.0, 0.0;

  // The same rotation matrix as before; theta radians around Z axis
  transform_2.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  // Print the transformation
  printf ("\nMethod #2: using an Affine3f\n");
  std::cout << transform_2.matrix() << std::endl;


  // Executing the transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // You can either apply transform_1 or transform_2; they are the same
  pcl::transformPointCloud (*cloud_source, *transformed_cloud, transform_2);

  // Visualization
  printf(  "\nPoint cloud colors :  white  = original point cloud\n"
      "                        red  = transformed point cloud\n");
  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");

   // Define R,G,B colors for the point cloud
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud_source, 255, 255, 255);
  // We add the point cloud to the viewer and pass the color handler
  viewer.addPointCloud (cloud_source, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_cloud_color_handler (transformed_cloud, 230, 20, 20); // Red
  viewer.addPointCloud (transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

}   




