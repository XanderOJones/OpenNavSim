#ifndef INS_MECHANIZATION_BASE_H
#define INS_MECHANIZATION_BASE_H

#include "../IMUSpec.h"
#include "../State.h"
#include "IMUMeasurement.h"

struct PVAState
{
    Eigen::Vector3d position;  // Position
    Eigen::Vector3d velocity;  // Velocity
    Eigen::Matrix3d attitude;   // Attitude
};

// Interface for INS mechanization in any frame
class INSMechanizationBase {
public:
    virtual ~INSMechanizationBase() = default;

    // Update state with IMU measurement and timestep
    virtual PVAState update(const IMUMeasurement& imu, 
                const Eigen::Vector3d& r_t__tb_in, 
                const Eigen::Vector3d& v_t__tb_in, 
                const Eigen::Matrix3d& C_t__b_in, 
                double dt) = 0;

    // Get current mechanized state
    virtual State getState() const = 0;

    // Reset state to a new value
    virtual void reset(const State& new_state) = 0;
};

#endif // INS_MECHANIZATION_BASE_H