#ifndef STATE_H
#define STATE_H

#include <Eigen/Dense>

class State {
public:
    // All state vectors (x) are assumed column vectors

    Eigen::VectorXd x;     // State vector (e.g. position, velocity, attitude, etc.)
    double timestamp;      // Time associated with this state (seconds)

    // Default constructor initializes with empty measurement and zero timestamp
    State() : timestamp(0.0) {}
    
    State(int state_size) : x(Eigen::VectorXd::Zero(state_size)), timestamp(0.0) {} // Optionally, construct with size

    int size() const { return x.size(); }      // Get dimension of the state vector

};

#endif
