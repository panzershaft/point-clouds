#include <iostream>
#include <string>
#include <sstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/impl/filter.hpp>
using namespace std;

std::string input_path = "/home/soham/lmr_project/build/";

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_one (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_two (new pcl::PointCloud<pcl::PointXYZ>);



void remove_nan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)
{
  //NaN removal
  std::vector<int> index1;
  pcl::removeNaNFromPointCloud(*cloud_in, *cloud_in, index1);
}

int main()
{
	
	for(int i = 1; i < 2; i++)
	{
	  //stringstream ss;
      std::stringstream sstream;
      sstream << input_path << "/frame" << i << ".pcd";
      string file1 = sstream.str();
      sstream.str(std::string());
      sstream << input_path << "/frame" << i + 9 << ".pcd";
      string file2 = sstream.str();
      sstream.str(std::string());
      //sstream.clear();
      cout << "\n--> " << file1 << "\n--> " << file2 << "\n";
      
      std::cout << "Loading: " << file1 << std::endl;
	  
      remove_nan(cloud_one);
      if (pcl::io::loadPCDFile(file1, *cloud_one) == -1)
      {
      
        PCL_ERROR("Couldn't read first file! \n"); // In case of file not being read
        return (-1);
      
      }

      //NaN removal
      remove_nan(cloud_two);
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
	
}   




