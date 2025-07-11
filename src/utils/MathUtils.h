#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include "Eigen/Dense"
#include "SimulationConstants.h"

// Converts 3x3 skew-symmetric matrix to 3x1 vector
inline Eigen::Vector3d ss2vec(const Eigen::Matrix3d& S) {
    return Eigen::Vector3d(S(2,1), S(0,2), S(1,0));
}

// Creates skew-symmetric matrix from 3x1 vector
inline Eigen::Matrix3d vec2ss(const Eigen::Vector3d& v) {
    Eigen::Matrix3d S;
    S <<  0,    -v(2),  v(1),
          v(2),  0,    -v(0),
         -v(1), v(0),   0;
    return S;
}

inline Eigen::Matrix3d llh2DCM_ECEF2TANG(const Eigen::Vector3d& llh) {
    // Convert latitude, longitude, height to radians
    double lat = llh(0) * PI / 180.0;
    double lon = llh(1) * PI / 180.0;

    double sL = sin(lat);
    double cL = cos(lat);
    double sl = sin(lon);
    double cl = cos(lon);

    // Compute DCM from LLH (eq. 2.158)
    Eigen::Matrix3d C_e__t;
    C_e__t << -sL*cl, -sl, -cL*cl,
              -sL*sl, cl, -cL*sl,
              cL, 0, -sL;

    return C_e__t;
}

inline Eigen::Vector3d llh2xyz_ECEF2TANG(const Eigen::Vector3d& llh) {
    // Convert latitude, longitude, height to radians
    double lat = llh(0) * PI / 180.0;
    double lon = llh(1) * PI / 180.0;
    double h = llh(2);

    double e = ECCENTRICITY;

    // eq. 2.106
    double RE = R0 / sqrt(1 - e*e * sin(lat) * sin(lat));

    // eq. 2.112
    Eigen::Vector3d xyz;
    xyz(0) = (RE + h) * cos(lat) * cos(lon);
    xyz(1) = (RE + h) * cos(lat) * sin(lon);
    xyz(2) = ((1 - e*e) * RE + h) * sin(lat);

    return xyz;
}

inline Eigen::Vector4d dcm2q(const Eigen::Matrix3d& C) {

    Eigen::Vector4d q;
    // Robust VectorNav-style method
    double q1s = 0.25 * (1 + C(0,0) - C(1,1) - C(2,2));
    double q2s = 0.25 * (1 - C(0,0) + C(1,1) - C(2,2));
    double q3s = 0.25 * (1 - C(0,0) - C(1,1) + C(2,2));
    double q4s = 0.25 * (1 + C(0,0) + C(1,1) + C(2,2));

    Eigen::Vector4d q_s_temp(q1s, q2s, q3s, q4s);
    Eigen::Index ind;
    q_s_temp.maxCoeff(&ind);

    switch (ind) {
        case 0: {
            double q1 = std::sqrt(q1s);
            double x = 4.0 * q1s;
            double y = C(0,1) + C(1,0);
            double z = C(2,0) + C(0,2);
            double w = C(1,2) - C(2,1);
            q = (1.0 / (4.0 * q1)) * Eigen::Vector4d(w, x, y, z);
            break;
        }
        case 1: {
            double q2 = std::sqrt(q2s);
            double x = C(0,1) + C(1,0);
            double y = 4.0 * q2s;
            double z = C(1,2) + C(2,1);
            double w = C(2,0) - C(0,2);
            q = (1.0 / (4.0 * q2)) * Eigen::Vector4d(w, x, y, z);
            break;
        }
        case 2: {
            double q3 = std::sqrt(q3s);
            double x = C(2,0) + C(0,2);
            double y = C(1,2) + C(2,1);
            double z = 4.0 * q3s;
            double w = C(0,1) - C(1,0);
            q = (1.0 / (4.0 * q3)) * Eigen::Vector4d(w, x, y, z);
            break;
        }
        case 3: {
            double q4 = std::sqrt(q4s);
            double x = C(1,2) - C(2,1);
            double y = C(2,0) - C(0,2);
            double z = C(0,1) - C(1,0);
            double w = 4.0 * q4s;
            q = (1.0 / (4.0 * q4)) * Eigen::Vector4d(w, x, y, z);
            break;
        }
    }

    // Apply sign convention [1;-1;-1;-1]
    q = q.normalized();
    q(1) *= -1;
    q(2) *= -1;
    q(3) *= -1;

    return q;
}

#endif // MATH_UTILS_H
