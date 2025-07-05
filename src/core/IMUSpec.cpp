#include "IMUSpec.h"
#include <include/json.hpp>
#include <fstream>

using json = nlohmann::json;

/*
IMUSpec JSON expected units:

Gyro parameters:
----------------
ARW                : deg / sqrt(hr)        (angle random walk)
b_g_BI_sigma       : rad/s                 (steady-state stddev of bias instability)
b_g_FB             : rad/s                 (fixed bias)
b_g_BS             : rad/s                 (bias stability bias)
M_g                : dimensionless         (misalignment matrix)
G_g                : dimensionless         (accel sensitivity matrix)
BI.correlation_time: seconds               (bias instability correlation time)

Accel parameters:
-----------------
VRW                : m/s² / sqrt(Hz)       (velocity random walk)
b_a_BI_sigma       : m/s²                  (steady-state stddev of bias instability)
b_a_FB             : m/s²                  (fixed bias)
b_a_BS             : m/s²                  (bias stability bias)
M_a                : dimensionless         (misalignment matrix)
BI.correlation_time: seconds               (bias instability correlation time)

Notes:
------
- ARW is converted internally to rad / sqrt(s) in code before computing per-sample sigma_g.
- VRW is converted internally to per-sample stddev using sqrt(Fs).
- All bias values and bias instability sigmas are in final units used by simulator (rad/s for gyro, m/s² for accel).
- All matrices are 3x3 stored in row-major flat array of 9 values.
*/

IMUSpec IMUSpec::fromJson(const std::string& filename) {
    IMUSpec spec;

    std::ifstream f(filename);
    json j;
    f >> j;

    spec.ARW = j["gyro"]["ARW"];
    spec.VRW = j["accel"]["VRW"];
    spec.gyro_bi_sigma = j["gyro"]["b_g_BI_sigma"];
    spec.accel_bi_sigma = j["accel"]["b_a_BI_sigma"];
    spec.gyro_BI_Tc = j["gyro"]["BI"]["correlation_time"];
    spec.accel_BI_Tc = j["accel"]["BI"]["correlation_time"];

    for (int i = 0; i < 3; ++i) {
        spec.gyro_FB(i) = j["gyro"]["b_g_FB"][i];
        spec.gyro_BS(i) = j["gyro"]["b_g_BS"][i];
        spec.accel_FB(i) = j["accel"]["b_a_FB"][i];
        spec.accel_BS(i) = j["accel"]["b_a_BS"][i];
    }

    auto toMatrix = [](const json& arr) {
        Eigen::Matrix3d m;
        for (int i = 0; i < 3; ++i)
            for (int k = 0; k < 3; ++k)
                m(i, k) = arr[i * 3 + k];
        return m;
    };

    spec.M_g = toMatrix(j["gyro"]["M_g"]);
    spec.G_g = toMatrix(j["gyro"]["G_g"]);
    spec.M_a = toMatrix(j["accel"]["M_a"]);

    return spec;
}