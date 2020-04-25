#include <iostream>

#include"matrix_transform.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/impl/filter.hpp>
using namespace std;
matrix_transform::PointCloudT::Ptr cloud_one (new pcl::PointCloud<pcl::PointXYZ>);
matrix_transform::PointCloudT::Ptr cloud_two (new pcl::PointCloud<pcl::PointXYZ>);


void matrix_transform::remove_nan(matrix_transform::PointCloudT::Ptr cloud_in)
{
  //NaN removal
  std::vector<int> index1;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, index1);
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

      //NaN removal
      matrix_transform::remove_nan(cloud_two);
      std::cout << "Loading: " << file2 << std::endl;
      if (pcl::io::loadPCDFile(file2, *cloud_two) == -1)
      {
        PCL_ERROR("Couldn't read the second file! \n"); // In case of file not being read
        return (-1);
      }
      
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
	
}   




