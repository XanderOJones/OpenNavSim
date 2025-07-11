#ifndef SIMULATION_CONSTANTS_H
#define SIMULATION_CONSTANTS_H

#include <Eigen/Dense>

// WGS-84 constants
constexpr double W_IE = 72.92115167e-6;         // Earth rotation rate (rad/s)
constexpr double MU = 3.986004418e14;           // Earth's gravitational constant (m^3/s^2)
constexpr double J2 = 1.082627e-3;              // Earth's second gravitational constant
constexpr double R0 = 6378137.0;                // Earth's equatorial radius (m)
constexpr double RP = 6356752.3142;             // Earth's polar radius (m)
constexpr double ECCENTRICITY = 0.0818191908425;// Eccentricity
constexpr double FLATTENING = 1.0 / 298.257223563; // Flattening
constexpr double PI = 3.14159265358979323846;

// Initial position constants (Default value set to New Mexico Tech campus)
namespace InitPos {
    constexpr double LAT_DEG = 34.0648;    // Latitude in degrees
    constexpr double LON_DEG = -106.9034;  // Longitude in degrees
    constexpr double HEIGHT_M = 0.0;      // Height in meters
}

// Returns Earth rotation rate vector in ECEF frame (rad/s)
inline Eigen::Vector3d earthRotationRateECEF() {
    return Eigen::Vector3d(0.0, 0.0, W_IE);
}

#endif // SIMULATION_CONSTANTS_H
