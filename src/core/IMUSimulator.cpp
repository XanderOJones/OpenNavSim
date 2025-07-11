#include "IMUSimulator.h"
#include <random>

constexpr double pi = 3.14159265358979323846;

// Constructor
IMUSimulator::IMUSimulator(const IMUSpec& spec, double Fs)
    : imu_spec(spec), Ts(1.0 / Fs),
      b_g_BI(Eigen::Vector3d::Zero()),
      b_a_BI(Eigen::Vector3d::Zero()),
      wk_g_last(Eigen::Vector3d::Zero()),
      wk_a_last(Eigen::Vector3d::Zero())
{}

// Generate measurement
void IMUSimulator::generateMeasurement(
    const Eigen::Vector3d& true_accel,
    const Eigen::Vector3d& true_gyro
) {
    updateFirstOrderMarkov(b_g_BI, wk_g_last,
                           imu_spec.gyro_bi_sigma,
                           imu_spec.gyro_BI_Tc);
    updateFirstOrderMarkov(b_a_BI, wk_a_last,
                           imu_spec.accel_bi_sigma,
                           imu_spec.accel_BI_Tc);

    Eigen::Vector3d b_g = b_g_BI + imu_spec.gyro_FB + imu_spec.gyro_BS;
    Eigen::Vector3d b_a = b_a_BI + imu_spec.accel_FB + imu_spec.accel_BS;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> nd(0.0, 1.0);

    Eigen::Vector3d w_g, w_a;
    double sigma_g = imu_spec.ARW * (pi/180.0) / 60.0 / sqrt(Ts);
    double sigma_a = imu_spec.VRW / sqrt(Ts);
    for (int i = 0; i < 3; ++i) {
        w_g(i) = sigma_g * nd(gen);
        w_a(i) = sigma_a * nd(gen);
    }

    Eigen::Vector3d noisy_gyro = b_g + (Eigen::Matrix3d::Identity() + imu_spec.M_g) * true_gyro
                 + imu_spec.G_g * true_accel + w_g;

    Eigen::Vector3d noisy_accel = b_a + (Eigen::Matrix3d::Identity() + imu_spec.M_a) * true_accel
                  + w_a;

    this->meas = IMUMeasurement(
        noisy_accel, noisy_gyro
    );
}

void IMUSimulator::updateFirstOrderMarkov(
    Eigen::Vector3d& b_BI,
    Eigen::Vector3d& wk_last,
    double sigma_BI,
    double Tc
) {
    double alpha = exp(-Ts / Tc);
    double Qd = sigma_BI * sigma_BI * (1.0 - exp(-2.0 * Ts / Tc));

    static thread_local std::mt19937 gen(std::random_device{}());
    std::normal_distribution<> nd(0.0, 1.0);

    Eigen::Vector3d wk;
    for (int i = 0; i < 3; ++i) {
        wk(i) = sqrt(Qd) * nd(gen);
    }

    // Bias update
    b_BI = alpha * b_BI + wk_last;
    wk_last = wk;
}
