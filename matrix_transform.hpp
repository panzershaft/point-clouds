
#ifndef MATRIX_TRANSFORM_HPP_INCLUDED
#define MATRIX_TRANSFORM_INCLUDED
#include <iostream>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/impl/filter.hpp>
using namespace std;

namespace matrix_transform
{
	typedef pcl::PointXYZ PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	std::string input_path = "/home/soham/lmr_project/build/";
	void remove_nan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);
	void lets_concat(pcl::PointCloud<pcl::PointXYZ>::Ptr a, pcl::PointCloud<pcl::PointXYZ>::Ptr b);
	int file_loader(int i);
}
#endif