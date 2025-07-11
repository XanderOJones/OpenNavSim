#ifndef TRAJECTORY_DATA_H
#define TRAJECTORY_DATA_H

#include <vector>
#include "../../include/Eigen/Dense"

// Holds trajectory data
class TrajectoryData {
public:
    std::vector<double> timestamps;                 // timestamps (s)
    std::vector<Eigen::Vector3d> positions;         // position (m)
    std::vector<Eigen::Vector3d> velocities;        // velocity (m/s)
    std::vector<Eigen::Vector3d> accelerations;     // acceleration (m/s²)
    std::vector<Eigen::Matrix3d> orientations;      // orientation DCMs (C_t__b)

    std::string coordinate_frame; // e.g. "ECEF", "ECI", "TANG"

    // Load data from 5 CSV files
    bool load(const std::string& timestamps_file,
              const std::string& position_file,
              const std::string& velocity_file,
              const std::string& acceleration_file,
              const std::string& orientation_file,
              const std::string& coord_frame);
};

#endif