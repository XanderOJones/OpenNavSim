#ifndef GROUND_TRUTH_GENERATOR_H
#define GROUND_TRUTH_GENERATOR_H

#include <Eigen/Dense>
#include <vector>
#include "SimulationConstants.h"

class GroundTruthGenerator {
public:
    GroundTruthGenerator();
    std::vector<Eigen::Vector3d> f_b__i_b;
    std::vector<Eigen::Vector3d> w_b__i_b;

    // Main generation method
    void computeGroundTruth(
        const std::vector<Eigen::Vector3d>& r_t__t_b,
        const std::vector<Eigen::Matrix3d>& C_t__b,
        const std::vector<Eigen::Vector3d>& v_t__t_b,
        const std::vector<Eigen::Vector3d>& a_t__t_b,
        const std::vector<double>& timestamps);

};

#endif // GROUND_TRUTH_GENERATOR_H
