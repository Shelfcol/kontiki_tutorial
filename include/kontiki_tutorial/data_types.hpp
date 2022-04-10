#ifndef DATA_TYPE_HPP_
#define DATA_TYPE_HPP_

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <mutex>

using namespace std;
using PointT = pcl::PointXYZI;
using CloudT = pcl::PointCloud<PointT>;

struct LidarData
{
    double stamp;
    CloudT::Ptr cloud;
};

struct LidarFrame
{
    double stamp;
    Eigen::Matrix4d T; // 与上一帧的相对运动
    Eigen::Matrix4d gT;
    CloudT::Ptr cloud{nullptr};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct ImuData
{
    double stamp;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyr; 
    Eigen::Quaterniond rot;
    Eigen::Quaterniond continous_quat; //由kontiki轨迹获得
    Eigen::Vector3d pos; //由kontiki轨迹获得

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};



#endif /* CC8BAE19_06EC_40CC_A471_DD4614BEA772 */
