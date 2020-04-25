
#ifndef MATRIX_TRANSFORM_HPP_INCLUDED
#define MATRIX_TRANSFORM_INCLUDED
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/impl/filter.hpp>


namespace matrix_transform
{
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	std::string input_path = "/home/soham/lmr_project/build/";
	void remove_nan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
	int file_loader(int i);
}
#endif