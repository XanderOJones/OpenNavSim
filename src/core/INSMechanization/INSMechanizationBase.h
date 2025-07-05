#ifndef INS_MECHANIZATION_BASE_H
#define INS_MECHANIZATION_BASE_H

#include "IMUSpec.h"
#include "State.h"
#include "IMUMeasurement.h"

// Interface for INS mechanization in any frame
class INSMechanizationBase {
public:
    virtual ~INSMechanizationBase() = default;

    // Update state with IMU measurement and timestep
    virtual void update(const IMUMeasurement& imu, double dt) = 0;

    // Get current mechanized state
    virtual State getState() const = 0;

    // Reset state to a new value
    virtual void reset(const State& new_state) = 0;
};

#endif // INS_MECHANIZATION_BASE_H