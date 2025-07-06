#ifndef NAV_UTILS_H
#define NAV_UTILS_H

#include "Eigen/Dense"
#include "SimulationConstants.h"

inline Eigen::Vector3d gamma_x__ib(const Eigen::Vector3d& r_e__eb) {

    double r_x__ib_norm = r_e__eb.norm();
    double t_temp = (r_e__eb(2) / r_x__ib_norm) * (r_e__eb(2) / r_x__ib_norm);
    double r_temp_x = (1 - 5 * t_temp) * r_e__eb(0);
    double r_temp_y = (1 - 5 * t_temp) * r_e__eb(1);
    double r_temp_z = (3 - 5 * t_temp) * r_e__eb(2);
    Eigen::Vector3d r_temp(r_temp_x, r_temp_y, r_temp_z);
    Eigen::Vector3d gamma_x__ib = (-MU/(r_x__ib_norm*r_x__ib_norm*r_x__ib_norm)) 
                                * (r_e__eb + (3/2)*J2*(R0*R0/(r_x__ib_norm*r_x__ib_norm))*r_temp);

    // Return the computed gamma vector
    return gamma_x__ib;
}




#endif