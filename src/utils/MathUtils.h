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
    double lat = llh(0) * M_PI / 180.0;
    double lon = llh(1) * M_PI / 180.0;

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
    double lat = llh(0) * M_PI / 180.0;
    double lon = llh(1) * M_PI / 180.0;
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

#endif // MATH_UTILS_H
