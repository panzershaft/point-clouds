/*    
 *     
 * Created by Soham 26/04/2020
 *   
 */

#include"matrix_transform.hpp"


matrix_transform::PointCloudT::Ptr cloud_one(new pcl::PointCloud < pcl::PointXYZ > );
matrix_transform::PointCloudT::Ptr cloud_two(new pcl::PointCloud < pcl::PointXYZ > );
matrix_transform::PointCloudT::Ptr cloud_source(new pcl::PointCloud < pcl::PointXYZ > );
matrix_transform::PointCloudT cloud_c;

void matrix_transform::remove_nan(matrix_transform::PointCloudT::Ptr cloud_in) {
  //NaN removal
  std::vector < int > index1;
  pcl::removeNaNFromPointCloud( * cloud_in, * cloud_in, index1);
}

void matrix_transform::lets_concat(matrix_transform::PointCloudT::Ptr cloud_a,
  matrix_transform::PointCloudT::Ptr cloud_b) {
  cloud_c = * cloud_a;
  cloud_c += * cloud_b;
  std::cerr << "Cloud C: " << std::endl;
  pcl::io::savePCDFileASCII("cloud_concatenate.pcd", cloud_c);
  std::cerr << "Saved " << cloud_c.points.size() << " data points to cloud_concatenate.pcd." << std::endl;
}

int matrix_transform::file_loader(int i) {
  //stringstream ss;
  std::stringstream sstream;
  sstream << matrix_transform::input_path << "/frame" << i << ".pcd";
  string file1 = sstream.str();
  sstream.str(std::string());
  sstream << matrix_transform::input_path << "/frame" << i + 9 << ".pcd";
  string file2 = sstream.str();
  sstream.str(std::string());

  std::cout << "Loading: " << file1 << std::endl;

  
  if (pcl::io::loadPCDFile(file1, * cloud_one) == -1) {

    PCL_ERROR("Couldn't read first file! \n"); // In case of file not being read
    return (-1);

  }
  matrix_transform::remove_nan(cloud_one);

  
  std::cout << "Loading: " << file2 << std::endl;
  if (pcl::io::loadPCDFile(file2, * cloud_two) == -1) {
    PCL_ERROR("Couldn't read the second file! \n"); // In case of file not being read
    return (-1);
  }
  matrix_transform::remove_nan(cloud_two);
  //now we add the two point clouds
  // matrix_transform::lets_concat(cloud_one, cloud_two);

  sstream.clear();
  file1.clear();
  file2.clear();
}

int main() {

  for (int i = 1; i < 2; i++) {
    matrix_transform::file_loader(i);

  }

  if (pcl::io::loadPCDFile("/home/soham/lmr_project/build/frame1.pcd", * cloud_source) == -1) {
    PCL_ERROR("Couldn't read first file! \n"); // In case of file not being read
    return (-1);
  }
  float theta = M_PI / 4; // The angle of rotation in radians 
  Eigen::Affine3f transformer = Eigen::Affine3f::Identity();

  transformer.translation() << 3.0, 0.0, 0.0;

  // The same rotation matrix as before; theta radians around Z axis
  transformer.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));


  printf("\nMethod #2: using an Affine3f\n");
  std::cout << transformer.matrix() << std::endl;

  pcl::PointCloud < pcl::PointXYZ > ::Ptr transformed_cloud(new pcl::PointCloud < pcl::PointXYZ > ());
  
  pcl::transformPointCloud( * cloud_source, * transformed_cloud, transformer);

  // Visualization
  printf("\nPoint cloud colors :  white  = original point cloud\n"
    "                        red  = transformed point cloud\n");
  pcl::visualization::PCLVisualizer viewer("Matrix transformation example");

  pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > source_cloud_color_handler(cloud_source, 255, 255, 255);
  viewer.addPointCloud(cloud_source, source_cloud_color_handler, "original_cloud");

  pcl::visualization::PointCloudColorHandlerCustom < pcl::PointXYZ > transformed_cloud_color_handler(transformed_cloud, 230, 20, 20); // Red
  viewer.addPointCloud(transformed_cloud, transformed_cloud_color_handler, "transformed_cloud");

  viewer.addCoordinateSystem(1.0, "original_cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); 
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "transformed_cloud");
  

  while (!viewer.wasStopped()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce();
  }

}