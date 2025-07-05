#include <iostream>
#include <Eigen/Dense>
#include "IMUSpec.h"
#include "IMUSimulator.h"
#include "TrajectoryData.h"

int main() {
    // Load IMU spec
    IMUSpec imu_spec = IMUSpec::fromJson("sensors/Orientus_IMU_calibrated.json");

    // Set sampling frequency
    double Fs = 100.0;
    double Ts = 1.0 / Fs;

    // Initialize IMU simulator
    IMUSimulator imu_sim(imu_spec, Fs);

    // Load trajectory
    TrajectoryData traj;
    if (!traj.load("timestamps.csv", "positions.csv", "velocities.csv",
                   "accelerations.csv", "orientations.csv", "TANG")) {
        std::cerr << "Failed to load trajectory files.\n";
        return 1;
    }

    // Initialize state
    Eigen::Vector3d position_est = traj.positions[0];
    Eigen::Vector3d velocity_est = traj.velocities[0];

    // Dead reckoning loop
    for (size_t i = 0; i < traj.timestamps.size(); ++i) {
        Eigen::Vector3d true_accel = traj.orientations[i].transpose() * traj.accelerations[i];
        Eigen::Vector3d true_gyro = Eigen::Vector3d::Zero();  // Assuming no rotation for simplicity

        Eigen::Vector3d noisy_accel, noisy_gyro;
        imu_sim.generateMeasurement(true_accel, true_gyro, noisy_accel, noisy_gyro);

        // Dead reckoning integration
        velocity_est += (traj.orientations[i] * noisy_accel) * Ts;
        position_est += velocity_est * Ts;
    }

    // Compute final position error
    Eigen::Vector3d position_true_end = traj.positions.back();
    double position_error = (position_est - position_true_end).norm();

    std::cout << "Final position error magnitude: " << position_error << " meters\n";

    return 0;
}