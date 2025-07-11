#include <iostream>
#include <memory>
#include <Eigen/Dense>

#include "TrajectoryData.h"
#include "IMUSpec.h"
#include "IMUSimulator.h"
#include "TangentialFrameMechanization.h"
#include "SimulationConstants.h"
#include "GroundTruthGenerator.h"

int main() {

    std::string timestamps_path = "trajectories/one_10k_loop_corrected_csv_timestamps.csv";
    std::string position_path = "trajectories/one_10k_loop_corrected_csv_r_t__t_b.csv";
    std::string velocity_path = "trajectories/one_10k_loop_corrected_csv_v_t__t_b.csv";
    std::string acceleration_path = "trajectories/one_10k_loop_corrected_csv_a_t__t_b.csv";
    std::string orientation_path = "trajectories/one_10k_loop_corrected_csv_C_t__b.csv";
    std::string coordinate_frame = "TANG";



    // Load trajectory (ground truth)
    TrajectoryData trajectory;
    if (!trajectory.load(timestamps_path,
                         position_path,
                         velocity_path, 
                         acceleration_path,
                         orientation_path,
                         coordinate_frame)) {
        std::cerr << "Failed to load trajectory.\n";
        return -1;
    }

    // Load IMU spec
    IMUSpec imu_spec;
    try {
        imu_spec = IMUSpec::fromJson("path/to/imu_spec.json");
    } catch (const std::exception& e) {
        std::cerr << "Failed to load IMU spec: " << e.what() << "\n";
        return -1;
    }

    // Compute synthetic IMU measurements from trajectory
    GroundTruthGenerator gt;
    gt.computeGroundTruth(
        trajectory.positions,
        trajectory.orientations,
        trajectory.velocities,
        trajectory.accelerations,
        trajectory.timestamps
    );

    // Initialize INS state to ground truth
    Eigen::Vector3d r_t__tb_INS = trajectory.positions[0];
    Eigen::Vector3d v_t__tb_INS = trajectory.velocities[0];
    Eigen::Matrix3d C_t__b_INS = trajectory.orientations[0];

    // Initialize INS mechanization state
    State state(15); // 15-state vector
    state.setStateVector(Eigen::VectorXd::Zero(15));

    // Initialize simulator and mechanization
    IMUSimulator imu_sim(imu_spec, imu_spec.sample_rate);
    TangentialFrameMechanization mech(state, imu_spec);

    size_t N = trajectory.positions.size();

    for (size_t i = 1; i < N; ++i) {
        double dt = trajectory.timestamps[i] - trajectory.timestamps[i - 1];

        // Ground truth values from GT generator
        Eigen::Vector3d accel_true = gt.f_b__i_b[i - 1];
        Eigen::Vector3d gyro_true = gt.w_b__i_b[i - 1];

        // Simulate IMU measurements with noise
        Eigen::Vector3d noisy_accel, noisy_gyro;
        imu_sim.generateMeasurement(accel_true, gyro_true);


        // Run mechanization step using noisy inputs
        mech.update(imu_sim.meas, r_t__tb_INS, v_t__tb_INS, C_t__b_INS, dt);
    }

    // Final ground truth
    Eigen::Vector3d r_t__tb_true = trajectory.positions.back();
    Eigen::Matrix3d C_t__b_true = trajectory.orientations.back();

    // Final position error
    Eigen::Vector3d pos_error = r_t__tb_INS - r_t__tb_true;

    // Final orientation error (trace-based angle metric)
    Eigen::Matrix3d C_err = C_t__b_INS.transpose() * C_t__b_true;
    double orientation_error_rad = std::acos((C_err.trace() - 1.0) / 2.0);

    // Output
    std::cout << "Final position error (m): " << pos_error.norm() << std::endl;
    std::cout << "Final orientation error (deg): " << orientation_error_rad * 180.0 / M_PI << std::endl;

    return 0;
}
