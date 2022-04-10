#include "kontiki_tutorial/kontiki_imu_test.hpp"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "kontiki_learning");

    SO3Test SO3_test;
    SO3_test.ReadImuData();
    SO3_test.Test();

    return 0;
}