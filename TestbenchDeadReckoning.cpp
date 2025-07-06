#include <iostream>
#include <memory>
#include <Eigen/Dense>

#include "TrajectoryData.h"
#include "IMUSpec.h"
#include "IMUSimulator.h"
#include "TangentialFrameMechanization.h"
#include "SimulationConstants.h"

int main() {
    // Load trajectory
    TrajectoryData trajectory;
    if (!trajectory.load("path/to/position.csv", "path/to/velocity.csv", "path/to/orientation.csv")) {
        std::cerr << "Failed to load trajectory.\n";
        return -1;
    }

    // Load IMU spec
    IMUSpec imu_spec;
    if (!imu_spec.loadFromJSON("path/to/imu_spec.json")) {
        std::cerr << "Failed to load IMU spec.\n";
        return -1;
    }

    // Initialize simulator and mechanization
    IMUSimulator imu_sim(imu_spec);
    TangentialFrameMechanization mech;

    // Initialize state
    Eigen::Vector3d r_t__tb = trajectory.positions[0];
    Eigen::Vector3d v_t__tb = trajectory.velocities[0];
    Eigen::Matrix3d C_t__b = trajectory.orientations[0];

    double dt = trajectory.time_step;
    size_t N = trajectory.positions.size();

    for (size_t i = 1; i < N; ++i) {
        // Get true motion
        Eigen::Vector3d true_accel = trajectory.accelerations[i];
        Eigen::Vector3d true_gyro = trajectory.angular_rates[i];

        // Generate noisy measurements
        Eigen::Vector3d noisy_accel, noisy_gyro;
        imu_sim.generateMeasurement(true_accel, true_gyro, noisy_accel, noisy_gyro);

        // Update mechanization
        mech.update(noisy_accel, noisy_gyro, r_t__tb, v_t__tb, C_t__b, dt);
    }

    // Compute final position error
    Eigen::Vector3d r_t__tb_true = trajectory.positions.back();
    Eigen::Vector3d pos_error = r_t__tb - r_t__tb_true;

    // Compute final orientation error (trace-based angle error)
    Eigen::Matrix3d C_err = C_t__b.transpose() * trajectory.orientations.back();
    double orientation_error_rad = std::acos((C_err.trace() - 1.0) / 2.0);

    // Output results
    std::cout << "Final position error (m): " << pos_error.norm() << std::endl;
    std::cout << "Final orientation error (deg): " << orientation_error_rad * 180.0 / M_PI << std::endl;

    return 0;
}
