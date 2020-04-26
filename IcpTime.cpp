#include"IcpTime.hpp"

IcpTime::IcpTime() {}

IcpTime::~IcpTime() 
{
    cout<< "running after delete called" << endl;
    cout<< "(Typically clean-up code)" << endl;
}

void IcpTime::remove_nan(IcpTime::PointCloudT::Ptr cloud_in) {
  //NaN removal
  std::vector < int > index1;
  pcl::removeNaNFromPointCloud( * cloud_in, * cloud_in, index1);
}

int IcpTime::ICPalgorithm(IcpTime::PointCloudT::Ptr cloud_icp, 
    IcpTime::PointCloudT::Ptr cloud_tr){

        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_icp);
        icp.setInputTarget(cloud_tr);
        
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);

        std::cout << "has converged:" << icp.hasConverged() << " score: " <<
        icp.getFitnessScore() << std::endl;
        std::cout << icp.getFinalTransformation() << std::endl;
        return (0);

    }

int IcpTime::file_loader(){
    IcpTime::PointCloudT::Ptr cloud_one(new IcpTime::PointCloudT);
    IcpTime::PointCloudT::Ptr cloud_two(new IcpTime::PointCloudT);
    IcpTime::PointCloudT::Ptr cloud_model(new IcpTime::PointCloudT);
    
    if (pcl::io::loadPCDFile("/home/soham/lmr_project/build/cloud_a.pcd", *cloud_one) == -1) {
        PCL_ERROR("Couldn't read first file! \n"); // In case of file not being read
        return (-1);
    } 

    IcpTime::remove_nan(cloud_one);

    if (pcl::io::loadPCDFile("/home/soham/lmr_project/build/cloud_b.pcd", *cloud_two) == -1) {
        PCL_ERROR("Couldn't read first file! \n"); // In case of file not being read
        return (-1);
    } 

    IcpTime::remove_nan(cloud_two);  

    *cloud_model = *cloud_one;
    IcpTime::ICPalgorithm(cloud_model, cloud_two);
}
