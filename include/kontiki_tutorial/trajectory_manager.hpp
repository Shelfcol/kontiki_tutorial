#ifndef TRAJECTORY_MANAGER_HPP_
#define TRAJECTORY_MANAGER_HPP_

#include <kontiki/sensors/constant_bias_imu.h>
#include <kontiki/sensors/vlp16_lidar.h>
#include <kontiki/trajectory_estimator.h>
#include <kontiki/trajectories/split_trajectory.h>
#include <kontiki/trajectories/uniform_r3_spline_trajectory.h>
#include <kontiki/trajectories/uniform_so3_spline_trajectory.h>
#include <kontiki/measurements/gyroscope_measurement.h>
#include <kontiki/measurements/accelerometer_measurement.h>
#include <kontiki/measurements/lidar_surfel_point.h>
#include <kontiki/measurements/orientation_measurement.h>
#include <kontiki/measurements/position_measurement.h>
#include <fstream>
#include <memory>
#include <vector>
#include <eigen3/Eigen/Dense>
#include "kontiki_tutorial/data_types.hpp"
#include "kontiki_tutorial/global_definition.h"

class TrajectoryManager
{
    using IMUSensor = kontiki::sensors::ConstantBiasImu;
    using SO3TrajEstimator = kontiki::TrajectoryEstimator<kontiki::trajectories::UniformSO3SplineTrajectory>;
    using R3TrajEstimator = kontiki::TrajectoryEstimator<kontiki::trajectories::UniformR3SplineTrajectory>;
    using SplitTrajEstimator = kontiki::TrajectoryEstimator<kontiki::trajectories::SplitTrajectory>;

    using GyroMeasurement = kontiki::measurements::GyroscopeMeasurement<IMUSensor>; // IMU的角速度观测
    using AccMeasurement = kontiki::measurements::AccelerometerMeasurement<IMUSensor>;
    using OrientationMeasurement = kontiki::measurements::OrientationMeasurement;
    using PositionMeasurement = kontiki::measurements::PositionMeasurement;

public:
    typedef std::shared_ptr<TrajectoryManager> Ptr;
    using Result = std::unique_ptr<kontiki::trajectories::TrajectoryEvaluation<double>>;

    explicit TrajectoryManager(double start_time,double end_time,
                                double knot_dist = 0.02,
                                double time_offset_padding =0)
            :time_offset_padding_(time_offset_padding),
            imu_(std::make_shared<IMUSensor>())
    {
        assert(knot_dist>0);
        double traj_start_time = start_time-time_offset_padding;
        double traj_end_time = end_time + time_offset_padding;
        traj_ = std::make_shared<kontiki::trajectories::SplitTrajectory>
                (knot_dist,knot_dist,traj_start_time,traj_start_time);
        Eigen::Quaterniond q0 = Eigen::Quaterniond::Identity();
        Eigen::Vector3d p0(0,0,0);
        traj_->SO3Spline()->ExtendTo(end_time,q0);
        traj_->R3Spline()->ExtendTo(end_time,p0);
    }
                                

    void initialSO3TrajWithGyro()
    {
        printf("initialSO3TrajWithGyro\n");
        std::shared_ptr<SO3TrajEstimator> estimate_SO3; 
        estimate_SO3= std::make_shared<SO3TrajEstimator>(traj_->SO3Spline()); // SO3样条
        addGyroscopeMeasurements(estimate_SO3);

        // 固定初始值
        double t0 = traj_->SO3Spline()->MinTime();
        Eigen::AngleAxisd rotation_vec(0.001,Eigen::Vector3d(0,0,1)); 
        Eigen::Quaterniond q0(rotation_vec.toRotationMatrix()); 
        // Eigen::Quaterniond q0(1,0,0,0);  //! jacobian不收敛
        auto m_q0 = std::make_shared<OrientationMeasurement>(t0,q0,gyro_weight_);
        estimate_SO3->AddMeasurement<OrientationMeasurement>(m_q0);

        // 四元数求解
        ceres::Solver::Summary summary = estimate_SO3->Solve(30,false);
        std::cout<<summary.BriefReport()<<std::endl;
    }

    void initialR3TrajWithAcc()
    {
        printf("initialR3TrajWithAcc\n");
        std::shared_ptr<R3TrajEstimator> estimate_R3;
        estimate_R3 = std::make_shared<R3TrajEstimator>(traj_->R3Spline());
        addAccelerometerMeasurements(estimate_R3);

        // 固定初始值
        double t0 = traj_->R3Spline()->MinTime();
        Eigen::Vector3d p0(0,0,0);
        auto m_p0 = std::make_shared<PositionMeasurement>(t0,p0,acc_weight_);
        estimate_R3->AddMeasurement<PositionMeasurement>(m_p0);

        ceres::Solver::Summary summary = estimate_R3->Solve(30,false);
        std::cout<<summary.BriefReport()<<std::endl;
    }

    // 获取每个IMU数据对应得到的姿态和位置,保存到imu_data_里面
    std::vector<ImuData> GetCorresOriAndPos()
    {
        for(auto& v:imu_data_){
            auto res_pos = traj_->Evaluate(v.stamp,kontiki::trajectories::EvalPosition);
            auto res_ori = traj_->Evaluate(v.stamp,kontiki::trajectories::EvalOrientation);
            v.continous_quat = res_ori->orientation;
            v.pos = res_pos->position;
        }
        return imu_data_;
    }
    // bias只有IMU时不输出
    void PrintImuInfo()
    {
        auto gravity = imu_->refined_gravity();
        auto gyro_bias = imu_->gyroscope_bias();
        auto acc_bias = imu_->accelerometer_bias();
        std::cout<<" gravity: "<<gravity<<std::endl;
        std::cout<<" gyro_bias: "<<gyro_bias<<std::endl;
        std::cout<<" acc_bias: "<<acc_bias<<std::endl;
    }

    void SetImuData(const std::vector<ImuData>& imu_data_vec){
        imu_data_ = imu_data_vec;
    }

private:
    template<typename TrajectoryModel>
    void addAccelerometerMeasurements(std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator)
    {
        acc_list_.clear();
        double weight = acc_weight_;
        const double min_time = estimator->trajectory()->MinTime();
        const double max_time = estimator->trajectory()->MaxTime();
        for(auto& v:imu_data_)
        {
            if(v.stamp<min_time||v.stamp>=max_time) {continue;}
            auto mg = std::make_shared<AccMeasurement>(imu_,v.stamp,v.acc,weight);
            acc_list_.push_back(mg);
            estimator->template AddMeasurement<AccMeasurement>(mg);
        }   
        printf("acc_list_ size = %d\n",(int)acc_list_.size());
    }

    template<typename TrajectoryModel>
    void addGyroscopeMeasurements(std::shared_ptr<kontiki::TrajectoryEstimator<TrajectoryModel>> estimator)
    {
        gyro_list_.clear();
        double weight = gyro_weight_;
        const double min_time = estimator->trajectory()->MinTime();
        const double max_time = estimator->trajectory()->MaxTime();
        for(auto& v:imu_data_)
        {
            if(v.stamp<min_time||v.stamp>=max_time) {continue;}
            auto mg = std::make_shared<GyroMeasurement>(imu_,v.stamp,v.gyr,weight);
            gyro_list_.push_back(mg);
            estimator->template AddMeasurement<GyroMeasurement>(mg);
        }   
        printf("gyro_list_ size = %d\n",(int)gyro_list_.size());
    }

private:
    std::shared_ptr<kontiki::trajectories::SplitTrajectory> traj_; // 定义一个轨迹指针
    std::shared_ptr<kontiki::sensors::ConstantBiasImu> imu_;
    std::vector< std::shared_ptr<GyroMeasurement>>  gyro_list_; // 保存指定时间段内的gyro值
    std::vector< std::shared_ptr<AccMeasurement>>  acc_list_; // 保存指定时间段内的gyro值

    double time_offset_padding_;
    double gyro_weight_{1.0};
    double acc_weight_{1.0};
    std::vector<ImuData> imu_data_;
};





#endif /* C5DCBC8F_1117_47C3_919F_9581F3F08C26 */
