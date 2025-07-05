#include "TrajectoryData.h"
#include <fstream>
#include <sstream>
#include <iostream>

// Helper: parse CSV row into vector of double
static std::vector<double> parseRow(const std::string& line) {
    std::vector<double> vals;
    std::stringstream ss(line);
    std::string field;
    while (std::getline(ss, field, ',')) {
        vals.push_back(std::stod(field));
    }
    return vals;
}

// Helper: load CSV file into rows x N matrix
static std::vector<std::vector<double>> parseMatrixCsv(const std::string& file, size_t expectedRows) {
    std::ifstream f(file);
    if (!f) {
        std::cerr << "Cannot open file: " << file << "\n";
        return {};
    }

    std::vector<std::vector<double>> matrix;
    std::string line;
    while (std::getline(f, line)) {
        auto row = parseRow(line);
        matrix.push_back(row);
    }

    if (matrix.size() != expectedRows) {
        std::cerr << "Expected " << expectedRows << " rows, got " << matrix.size() << " in " << file << "\n";
        return {};
    }

    // Ensure all rows have same column count
    size_t N = matrix[0].size();
    for (const auto& r : matrix) {
        if (r.size() != N) {
            std::cerr << "Inconsistent column count in file: " << file << "\n";
            return {};
        }
    }

    return matrix;
}

bool TrajectoryData::load(const std::string& timestamps_file,
                          const std::string& position_file,
                          const std::string& velocity_file,
                          const std::string& acceleration_file,
                          const std::string& orientation_file,
                          const std::string& coord_frame) {

    coordinate_frame = coord_frame;

    // Load timestamps: 1 x N
    auto t_data = parseMatrixCsv(timestamps_file, 1);
    if (t_data.empty()) return false;

    size_t N = t_data[0].size();
    for (double t : t_data[0]) {
        timestamps.push_back(t);
    }

    // Load position: 3 x N
    auto p_data = parseMatrixCsv(position_file, 3);
    if (p_data.empty() || p_data[0].size() != N) {
        std::cerr << "Position file size mismatch.\n";
        return false;
    }
    for (size_t i = 0; i < N; ++i) {
        positions.emplace_back(p_data[0][i], p_data[1][i], p_data[2][i]);
    }

    // Load velocity: 3 x N
    auto v_data = parseMatrixCsv(velocity_file, 3);
    if (v_data.empty() || v_data[0].size() != N) {
        std::cerr << "Velocity file size mismatch.\n";
        return false;
    }
    for (size_t i = 0; i < N; ++i) {
        velocities.emplace_back(v_data[0][i], v_data[1][i], v_data[2][i]);
    }

    // Load acceleration: 3 x N
    auto a_data = parseMatrixCsv(acceleration_file, 3);
    if (a_data.empty() || a_data[0].size() != N) {
        std::cerr << "Acceleration file size mismatch.\n";
        return false;
    }
    for (size_t i = 0; i < N; ++i) {
        accelerations.emplace_back(a_data[0][i], a_data[1][i], a_data[2][i]);
    }

    // Load orientation: 9 x N
    auto o_data = parseMatrixCsv(orientation_file, 9);
    if (o_data.empty() || o_data[0].size() != N) {
        std::cerr << "Orientation file size mismatch.\n";
        return false;
    }
    for (size_t i = 0; i < N; ++i) {
        Eigen::Matrix3d dcm;
        dcm << o_data[0][i], o_data[3][i], o_data[6][i],
               o_data[1][i], o_data[4][i], o_data[7][i],
               o_data[2][i], o_data[5][i], o_data[8][i];
        orientations.push_back(dcm);
    }

    return true;
}
