#ifndef IMU_MEASUREMENT_H
#define IMU_MEASUREMENT_H

#include <Eigen/Dense>

// Struct representing IMU measurement at one epoch
struct IMUMeasurement {
    Eigen::Vector3d accel_mps2;  // Body-frame acceleration [m/s^2]
    Eigen::Vector3d gyro_rps;    // Body-frame angular rate [rad/s]

    IMUMeasurement()
        : accel_mps2(Eigen::Vector3d::Zero()),
          gyro_rps(Eigen::Vector3d::Zero()) {}

    IMUMeasurement(const Eigen::Vector3d& accel, const Eigen::Vector3d& gyro)
        : accel_mps2(accel), gyro_rps(gyro) {}
};

#endif // IMU_MEASUREMENT_H