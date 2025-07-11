#ifndef STATE_H
#define STATE_H

#include <Eigen/Dense>

class State {
public:
    Eigen::VectorXd x;     // State vector (e.g. position, velocity, attitude, etc.)
    double timestamp;      // Time associated with this state (seconds)

    // Constructors
    State() : timestamp(0.0) {}
    State(int state_size) : x(Eigen::VectorXd::Zero(state_size)), timestamp(0.0) {}

    // Get dimension of the state vector
    int size() const { return x.size(); }

    // Retrieve the state vector
    const Eigen::VectorXd& getStateVector() const { return x; }

    // Update the state vector (by copy)
    void setStateVector(const Eigen::VectorXd& new_x) { x = new_x; }
};

#endif