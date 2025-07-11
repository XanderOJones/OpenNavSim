#include "GroundTruthGenerator.h"
#include "MathUtils.h"
#include "NavUtils.h"
#include "SimulationConstants.h"

GroundTruthGenerator::GroundTruthGenerator() {
    // Initialize vectors to empty
    f_b__i_b.clear();
    w_b__i_b.clear();
}

void GroundTruthGenerator::computeGroundTruth(
    const std::vector<Eigen::Vector3d>& r_t__t_b,
    const std::vector<Eigen::Matrix3d>& C_t__b,
    const std::vector<Eigen::Vector3d>& v_t__t_b,
    const std::vector<Eigen::Vector3d>& a_t__t_b,
    const std::vector<double>& timestamps)
{
    const int N = r_t__t_b.size();
    //const double dt = constants.dt;

    //v_t__t_b.resize(3, N);
    this->f_b__i_b.resize(N);
    this->w_b__i_b.resize(N);

    // Precompute constants
    Eigen::Vector3d llh_initial;
    llh_initial << InitPos::LAT_DEG, InitPos::LON_DEG, InitPos::HEIGHT_M;

    Eigen::Matrix3d Ohm_i__ie = vec2ss(earthRotationRateECEF());
    Eigen::Matrix3d C_e__t = llh2DCM_ECEF2TANG(llh_initial);
    Eigen::Matrix3d C_t__e = C_e__t.transpose();
    Eigen::Vector3d r_e__et = llh2xyz_ECEF2TANG(llh_initial);


    for (int k = 1; k < N; ++k) {
        Eigen::Matrix3d C_e__b = C_e__t * C_t__b[k];
        Eigen::Vector3d r_e__eb = r_e__et + C_e__t * r_t__t_b[k];
        Eigen::Vector3d v_e__eb = C_e__t * v_t__t_b[k];
        Eigen::Vector3d a_e__eb = C_e__t * a_t__t_b[k];

        double dt = timestamps[k] - timestamps[k - 1];

        if (!C_t__b[k].isApprox(Eigen::Matrix3d::Identity(), 1e-10)) {
            Eigen::Matrix3d C_bkm1__bk = C_t__b[k - 1].transpose() * C_t__b[k];
            Eigen::Vector4d q = dcm2q(C_bkm1__bk);
            Eigen::Vector3d ksin = q.tail<3>();
            Eigen::Vector3d kvec = ksin.normalized();
            double dtheta = 2.0 * acos(q(0));
            this->w_b__i_b[k - 1] = kvec * dtheta / dt +
                                   C_t__b[k].transpose() * C_e__t.transpose() * earthRotationRateECEF();
        } else {
            this->w_b__i_b[k - 1] = Eigen::Vector3d::Zero() +
                                   C_t__b[k].transpose() * C_e__t.transpose() * earthRotationRateECEF();
        }

        Eigen::Vector3d a_e__ib = a_e__eb +
                                  2 * Ohm_i__ie * v_e__eb +
                                  Ohm_i__ie * Ohm_i__ie * r_e__eb;

        this->f_b__i_b[k - 1] = C_e__b.transpose() * (a_e__ib - gamma_x__ib(r_e__eb));
    }
}