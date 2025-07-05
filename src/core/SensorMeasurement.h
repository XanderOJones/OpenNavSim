#ifndef SENSOR_MEASUREMENT_H
#define SENSOR_MEASUREMENT_H

#include <Eigen/Dense>
#include <string>

struct SensorMeasurement {
    Eigen::VectorXd z;       // The measurement vector
    Eigen::MatrixXd R;       // Measurement noise covariance
    double timestamp;        // Time of measurement (seconds)
    std::string type;        // Type of sensor (e.g. "GPS", "IMU", "Barometer")

    // Default constructor initializes with empty measurement and zero timestamp
    SensorMeasurement() : timestamp(0.0) {}
};

#endif
