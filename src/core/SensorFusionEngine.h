#ifndef SENSOR_FUSION_ENGINE_H
#define SENSOR_FUSION_ENGINE_H

#include "State.h"
#include "SensorMeasurement.h"
#include <string>
#include <ostream>


class SensorFusionEngine {
public:
    virtual ~SensorFusionEngine() = default;

    // Subclasses must implement predict, update, get state, and reset methods

    // Predict the state forward by dt seconds
    virtual void predict(double dt) = 0;

    // Incorporate a measurement
    virtual void update(const SensorMeasurement& meas) = 0;

    // Get current state estimate
    virtual State getState() const = 0;

    // Must allow resetting to a defined state
    virtual void reset(const State& new_state) = 0;

    // Diagnostics and output stream methods are optional to implement, default behavior in SensorFusionEngine.cpp

    // Dump human-readable info about current state
    virtual std::string diagnostics() const;

    // Log to an output stream
    virtual void log(std::ostream& os) const;
};

#endif