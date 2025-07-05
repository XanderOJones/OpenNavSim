#ifndef TANGENTIAL_FRAME_MECHANIZATION_H
#define TANGENTIAL_FRAME_MECHANIZATION_H

#include "INSMechanizationBase.h"

// Tangential frame INS mechanization implementation
class TangentialFrameMechanization : public INSMechanizationBase {
public:
    TangentialFrameMechanization(const State& initial_state, const IMUSpec& imu_spec);

    // Update state with IMU measurement
    void update(const IMUMeasurement& imu, double dt) override;

    // Get current state estimate
    State getState() const override;

    // Reset to a specified state
    void reset(const State& new_state) override;

private:
    State state_;
    IMUSpec imu_spec_;

    // Internal helpers for mechanization math
    void integratePosition(double dt);
    void integrateVelocity(const IMUMeasurement& imu, double dt);
    void integrateAttitude(const IMUMeasurement& imu, double dt);
};

#endif // TANGENTIAL_FRAME_MECHANIZATION_H