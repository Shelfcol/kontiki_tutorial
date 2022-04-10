#ifndef KONTIKI_IMU_TEST_HPP_
#define KONTIKI_IMU_TEST_HPP_
#include "kontiki_tutorial/trajectory_manager.hpp"
#include "kontiki_tutorial/data_types.hpp"
#include "kontiki_tutorial/global_definition.h"
#include "kontiki_tutorial/plot_kitti_traj.hpp"
#include <vector>
#include <memory>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <yaml-cpp/yaml.h>
class SO3Test
{
public:
    void ReadImuData()
    {
        std::string config_file_path = WORK_SPACE_PATH+"/config/config.yaml";
        YAML::Node config_node = YAML::LoadFile(config_file_path);

        std::string bag_path = config_node["bag_path"].as<std::string>();
        std::string imu_topic_name = config_node["imu_topic_name"].as<std::string>();

        bag_.reset(new rosbag::Bag);
        bag_->open(bag_path, rosbag::bagmode::Read);

        std::vector<std::string> topics;
        topics.push_back(imu_topic_name);

        rosbag::View view;
        view.addQuery(*bag_);

        double begin_time = view.getBeginTime().toSec();
        double end_time = view.getEndTime().toSec();
        // end_time = (end_time-begin_time>5)?begin_time+5:end_time; // 只取5s
        printf("reading imu data\n");
        for(rosbag::MessageInstance const m:view)
        {
            const std::string& topic = m.getTopic();
            if(imu_topic_name == topic)
            {
                sensor_msgs::ImuConstPtr imu_msg = m.instantiate<sensor_msgs::Imu>();
                double time = imu_msg->header.stamp.toSec();
                if(time>end_time) {break;}
                ImuData imu_dat;
                imu_dat.stamp = time;
                imu_dat.acc = Eigen::Vector3d(imu_msg->linear_acceleration.x,imu_msg->linear_acceleration.y,imu_msg->linear_acceleration.z);
                imu_dat.gyr = Eigen::Vector3d(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);
                imu_data_.push_back(imu_dat);
            }
        }
        printf("imu data size = %d\n",(int)imu_data_.size());
    }

    void Test()
    {
        double start_time = imu_data_.front().stamp;
        double end_time = imu_data_.back().stamp;
        double knot_dist = 0.02;
        double time_offset_padding = 0;

        // 使用kontiki的连续轨迹获得
        traj_manager_ = std::make_shared<TrajectoryManager>(start_time,end_time,knot_dist,time_offset_padding);
        traj_manager_->SetImuData(imu_data_);
        traj_manager_->initialSO3TrajWithGyro();  
        traj_manager_->initialR3TrajWithAcc();

        imu_data_ = traj_manager_->GetCorresOriAndPos();
        SaveTumFormat(WORK_SPACE_PATH+"/data/");
        SaveKittiFormat(WORK_SPACE_PATH+"/data/");
        traj_manager_->PrintImuInfo();
        string trajectory_file = WORK_SPACE_PATH+"/data/imu_data_res_tum.txt";
        DrawTraj(trajectory_file);
    }

    void SaveKittiFormat(const string& map_path)
    {
        FILE *fp = NULL;
        char end1 = 0x0d; // "/n"
        char end2 = 0x0a;

        // lidar odometry
        string lidar_tum_file = map_path + "imu_data_res_kitti.txt";
        fp = fopen(lidar_tum_file.c_str(), "w+");

        if (fp == NULL)
        {
            printf("fail to open file %s ! \n", lidar_tum_file.c_str());
            exit(1);
        }
        else
            printf("KITTI : write lidar data to %s \n", lidar_tum_file.c_str());

        for (int i = 0; i < imu_data_.size(); ++i)
        {
            auto q = imu_data_[i].continous_quat;
            Eigen::Matrix3d R;
            R = q;

            Eigen::Vector3d t = imu_data_[i].pos;
            double time = imu_data_[i].stamp;
            fprintf(fp, "%.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf %.5lf%c",
                    R(0,0),R(0,1),R(0,2),t.x(),
                    R(1,0),R(1,1),R(1,2),t.y(),
                    R(2,0),R(2,1),R(2,2),t.z(),
                    end2);
        }
        fclose(fp);
    }

    void SaveTumFormat(const string& map_path)
    {
        FILE *fp = NULL;
        char end1 = 0x0d; // "/n"
        char end2 = 0x0a;

        // lidar odometry
        string lidar_tum_file = map_path + "imu_data_res_tum.txt";
        fp = fopen(lidar_tum_file.c_str(), "w+");

        if (fp == NULL)
        {
            printf("fail to open file %s ! \n", lidar_tum_file.c_str());
            exit(1);
        }
        else
            printf("TUM : write lidar data to %s \n", lidar_tum_file.c_str());

        for (size_t i = 0; i < imu_data_.size(); ++i)
        {
            Eigen::Quaterniond q = imu_data_[i].continous_quat;
            Eigen::Vector3d t = imu_data_[i].pos;
            double time = imu_data_[i].stamp;
            fprintf(fp, "%.3lf %.3lf %.3lf %.3lf %.5lf %.5lf %.5lf %.5lf%c",
                    time, t.x(), t.y(), t.z(),
                    q.x(), q.y(), q.z(), q.w(), end2);
        }
        fclose(fp);
    }

public:
    std::vector<ImuData> imu_data_;
    std::shared_ptr<rosbag::Bag> bag_;
    std::shared_ptr<TrajectoryManager> traj_manager_;
};

#endif /* KONTIKI_IMU_TEST_HPP_ */
