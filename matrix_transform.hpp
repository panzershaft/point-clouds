
#ifndef HELLO_HPP_INCLUDED
#define HELLO_HPP_INCLUDED



#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/impl/filter.hpp>

namespace matrix_transform
{
  void remove_nan(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);

}
#endif