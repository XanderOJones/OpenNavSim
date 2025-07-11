#ifndef TANGENTIAL_FRAME_MECHANIZATION_H
#define TANGENTIAL_FRAME_MECHANIZATION_H

#include "INSMechanizationBase.h"

// Tangential frame INS mechanization implementation
class TangentialFrameMechanization : public INSMechanizationBase {
public:
    TangentialFrameMechanization(const PVAState& initial_state, const IMUSpec& imu_spec);

    // Update state with IMU measurement
    PVAState update(const IMUMeasurement& imu, 
                const Eigen::Vector3d& r_t__tb_in, 
                const Eigen::Vector3d& v_t__tb_in, 
                const Eigen::Matrix3d& C_t__b_in, 
                double dt) override;

    // Get current state estimate
    PVAState getState() const override;

    // Reset to a specified state
    void reset(const PVAState& new_state) override;

private:
    PVAState state_;
    IMUSpec imu_spec_;
    Eigen::Matrix3d C_e__t; // DCM from ECEF to Tangential frame
    Eigen::Matrix3d C_t__e; // DCM from Tangential to ECEF frame
    Eigen::Matrix3d Ohm_i__ie; // Earth rotation rate in ECEF / ECI (equiv.)
    Eigen::Vector3d r_e__et; // Position in ECEF frame

    // Internal helpers for mechanization math
    void integratePosition(double dt);
    void integrateVelocity(const IMUMeasurement& imu, double dt);
    void integrateAttitude(const IMUMeasurement& imu, double dt);
};

#endif // TANGENTIAL_FRAME_MECHANIZATION_H