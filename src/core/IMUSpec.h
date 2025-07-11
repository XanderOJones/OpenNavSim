#ifndef IMU_SPEC_H
#define IMU_SPEC_H

#include <Eigen/Dense>

// Holds IMU specs loaded from JSON
struct IMUSpec {
    double ARW;
    double VRW;
    double gyro_bi_sigma;
    double accel_bi_sigma;
    double gyro_BI_Tc;
    double accel_BI_Tc;
    double sample_rate;

    Eigen::Vector3d gyro_FB;
    Eigen::Vector3d gyro_BS;
    Eigen::Vector3d accel_FB;
    Eigen::Vector3d accel_BS;

    Eigen::Matrix3d M_g;
    Eigen::Matrix3d G_g;
    Eigen::Matrix3d M_a;

    static IMUSpec fromJson(const std::string& filename);
};

#endif