#include <iostream>
#include <memory>
#include <fstream>

#include "Eigen/Dense"
#include "TrajectoryData.h"
#include "IMUSpec.h"
#include "IMUSimulator.h"
#include "TangentialFrameMechanization.h"
#include "SimulationConstants.h"
#include "GroundTruthGenerator.h"

int main() {

    std::string IMU_path = "X:/OpenNavSim/sensors/Orientus_IMU_calibrated.json";

    std::string timestamps_path = "X:/OpenNavSim/trajectories/one_10k_loop/one_10k_loop_corrected_csv_timestamps.csv";
    std::string position_path = "X:/OpenNavSim/trajectories/one_10k_loop/one_10k_loop_corrected_csv_r_t__t_b.csv";
    std::string velocity_path = "X:/OpenNavSim/trajectories/one_10k_loop/one_10k_loop_corrected_csv_v_t__t_b.csv";
    std::string acceleration_path = "X:/OpenNavSim/trajectories/one_10k_loop/one_10k_loop_corrected_csv_a_t__t_b.csv";
    std::string orientation_path = "X:/OpenNavSim/trajectories/one_10k_loop/one_10k_loop_corrected_csv_C_t__b.csv";
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
        imu_spec = IMUSpec::fromJson(IMU_path);
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
    PVAState state;  // Create an instance

    state.position = r_t__tb_INS;
    state.velocity = v_t__tb_INS;
    state.attitude = C_t__b_INS;
    //state.setStateVector(Eigen::VectorXd::Zero(15));

    // Initialize simulator and mechanization
    IMUSimulator imu_sim(imu_spec, imu_spec.sample_rate);
    TangentialFrameMechanization mech(state, imu_spec);

    size_t N = trajectory.positions.size();
    std::ofstream dr_file("estimated_positions.csv");
    

    //N = 5;

    for (size_t i = 1; i < N - 1; ++i) {
        double dt = trajectory.timestamps[i] - trajectory.timestamps[i - 1];

        // Ground truth values from GT generator
        Eigen::Vector3d accel_true = gt.f_b__i_b[i - 1];
        Eigen::Vector3d gyro_true = gt.w_b__i_b[i - 1];

        // Simulate IMU measurements with noise
        //Eigen::Vector3d noisy_accel, noisy_gyro;
        imu_sim.generateMeasurement(accel_true, gyro_true);
        //std::cout << "Noisy Accel: " << imu_sim.meas.accel_mps2.transpose() << "\n";
        //std::cout << "Noisy Gyro: " << imu_sim.meas.gyro_rps.transpose() << "\n";
        //std::cout << "True Accel: " << accel_true.transpose() << "\n";
        //std::cout << "True Gyro: " << gyro_true.transpose() << "\n";
        //std::cout << "Position (r_t__t_b_INS): " << r_t__tb_INS.transpose() << "\n";

        // Run mechanization step using noisy inputs
        mech.update(imu_sim.meas, r_t__tb_INS, v_t__tb_INS, C_t__b_INS, dt);
        r_t__tb_INS = mech.getState().position;
        v_t__tb_INS = mech.getState().velocity;
        C_t__b_INS = mech.getState().attitude;

        //std::cout << "imu_sim: " << imu_sim.meas.accel_mps2 << " , " << imu_sim.meas.gyro_rps << std::endl;

        

        dr_file << r_t__tb_INS.transpose() << "\n";
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
    std::cout << "Final orientation error (deg): " << orientation_error_rad * 180.0 / PI << std::endl;
    std::cout << "Initial Orientation: " << trajectory.orientations[0] << std::endl;


    // Export trajectories to CSV
    std::ofstream gt_file("true_positions.csv");


    for (size_t i = 0; i < N; ++i) {
        const auto& gt_pos = trajectory.positions[i];
        gt_file << gt_pos(0) << "," << gt_pos(1) << "," << gt_pos(2) << "\n";
    }

    // Only export the final estimated position for now (dead reckoned)
    

    gt_file.close();
    dr_file.close();


    return 0;
}
