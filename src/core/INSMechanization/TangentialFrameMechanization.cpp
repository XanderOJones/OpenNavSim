#include "TangentialFrameMechanization.h"
#include "MathUtils.h"
#include "NavUtils.h"
#include "SimulationConstants.h"
#include <iostream>

// Constructor sets initial state and IMU spec
TangentialFrameMechanization::TangentialFrameMechanization(const PVAState& initial_state,
                                                           const IMUSpec& imu_spec)
    : state_(initial_state), imu_spec_(imu_spec) {

    Eigen::Vector3d llh_initial;
    llh_initial << InitPos::LAT_DEG, InitPos::LON_DEG, InitPos::HEIGHT_M;

    Eigen::Matrix3d Ohm_i__ie = vec2ss(earthRotationRateECEF());
    Eigen::Matrix3d C_e__t = llh2DCM_ECEF2TANG(llh_initial);
    Eigen::Matrix3d C_t__e = C_e__t.transpose();
    Eigen::Vector3d r_e__et = llh2xyz_ECEF2TANG(llh_initial);
}

// Update state using mechanization equations
PVAState TangentialFrameMechanization::update(const IMUMeasurement& imu, 
                const Eigen::Vector3d& r_t__tb_in, 
                const Eigen::Vector3d& v_t__tb_in, 
                const Eigen::Matrix3d& C_t__b_in, 
                double dt) {
    // Preprocessing
    Eigen::Vector3d w_b__ib_tilde = imu.gyro_rps;
    Eigen::Vector3d f_b__ib_tilde = imu.accel_mps2;

    Eigen::Matrix3d Ohm_i__ie = vec2ss(earthRotationRateECEF());
    r_e__et = llh2xyz_ECEF2TANG(Eigen::Vector3d(InitPos::LAT_DEG, InitPos::LON_DEG, InitPos::HEIGHT_M));
    C_e__t = llh2DCM_ECEF2TANG(Eigen::Vector3d(InitPos::LAT_DEG, InitPos::LON_DEG, InitPos::HEIGHT_M));
    C_t__e = C_e__t.transpose();
    Eigen::Matrix3d Ohm_b__ib = vec2ss(w_b__ib_tilde);
    Eigen::Vector3d r_e__eb = r_e__et + C_e__t * r_t__tb_in;
    Eigen::Matrix3d Ohm_t__ie = C_t__e * Ohm_i__ie * C_t__e; // Note: Ohm_i__ie = Ohm_e__ie
    Eigen::Matrix3d Ohm_t__tb = C_t__b_in * Ohm_b__ib * C_t__b_in.transpose() - Ohm_t__ie;
    Eigen::Vector3d w_t__t_b = ss2vec(Ohm_t__tb);

    double dtheta = (w_t__t_b * dt).norm();
    Eigen::Vector3d k = w_t__t_b * dt / dtheta;
    Eigen::Matrix3d Kappa = vec2ss(k);

    // Attitude Update
    Eigen::Matrix3d C_t__b_out = (Eigen::Matrix3d::Identity() + sin(dtheta) * Kappa + (1-cos(dtheta)) * Kappa * Kappa) * C_t__b_in;

    // Specific Force Update
    Eigen::Matrix3d alpha_cross = vec2ss(w_b__ib_tilde) * dt;
    double alpha_cross_mag = ss2vec(alpha_cross).norm();
    Eigen::Matrix3d C_bm__b_bar = Eigen::Matrix3d::Identity()
        + ((1 - std::cos(alpha_cross_mag)) / (alpha_cross_mag * alpha_cross_mag)) * alpha_cross
        + (1.0 / (alpha_cross_mag * alpha_cross_mag)) * (1 - std::sin(alpha_cross_mag) / alpha_cross_mag)
        * (alpha_cross * alpha_cross);


    Eigen::Matrix3d C_t__b_bar = C_t__b_in * C_bm__b_bar - 0.5 * Ohm_t__ie * C_t__b_in * dt;
    Eigen::Vector3d f_t__ib = C_t__b_bar * f_b__ib_tilde;

    // Velocity Update
    Eigen::Vector3d gamma_e__ib = gamma_x__ib(r_e__eb);
    Eigen::Vector3d g_e__b = gamma_e__ib - Ohm_i__ie * Ohm_i__ie * r_e__eb;
    Eigen::Vector3d g_t__b = C_t__e * g_e__b;

    Eigen::Vector3d a_t__tb = f_t__ib + g_t__b - 2 * Ohm_t__ie * v_t__tb_in;
    Eigen::Vector3d v_t__tb_prime = v_t__tb_in + a_t__tb * dt;
    Eigen::Vector3d v_t__tb_out = v_t__tb_in + (f_t__ib + g_t__b - Ohm_t__ie * (v_t__tb_in + v_t__tb_prime)) * dt;

    // Position Update
    Eigen::Vector3d r_t__tb_out = r_t__tb_in + v_t__tb_out * dt + 0.5 * (a_t__tb) * dt * dt;

    //std::cout << "dtheta: " << dtheta << "\n";

    PVAState state_out;
    state_out.position = r_t__tb_out;
    state_out.velocity = v_t__tb_out;
    state_out.attitude = C_t__b_out;
    state_ = state_out;
    return state_out;
}

// Return current state
PVAState TangentialFrameMechanization::getState() const {
    return state_;
}

// Reset state to a new value
void TangentialFrameMechanization::reset(const PVAState& new_state) {
    state_ = new_state;
}

// Integrate position in tangential frame
void TangentialFrameMechanization::integratePosition(double dt) {
    // Implement tangential position update here
}

// Integrate velocity using accelerometer data
void TangentialFrameMechanization::integrateVelocity(const IMUMeasurement& imu, double dt) {
    // Implement tangential velocity update here
}

// Integrate attitude using gyro data
void TangentialFrameMechanization::integrateAttitude(const IMUMeasurement& imu, double dt) {
    // Implement tangential attitude update here
}