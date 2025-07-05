#ifndef IMU_SIMULATOR_H
#define IMU_SIMULATOR_H

#include <Eigen/Dense>
#include "IMUSpec.h"

// IMU simulator: generates accelerometer and gyro measurements with noise and bias
class IMUSimulator {
public:
    // Construct with IMU spec and sample rate
    IMUSimulator(const IMUSpec& spec, double Fs);

    // Generate noisy measurements
    void generateMeasurement(
        const Eigen::Vector3d& true_accel,
        const Eigen::Vector3d& true_gyro,
        Eigen::Vector3d& noisy_accel,
        Eigen::Vector3d& noisy_gyro
    );

private:
    double Ts;
    IMUSpec imu_spec;

    // Bias states
    Eigen::Vector3d b_g_BI;
    Eigen::Vector3d b_a_BI;
    Eigen::Vector3d wk_g_last;
    Eigen::Vector3d wk_a_last;

    // Update first-order Markov bias
    void updateFirstOrderMarkov(
        Eigen::Vector3d& b_BI,
        Eigen::Vector3d& wk_last,
        double sigma_BI,
        double Tc
    );
};

#endif