#ifndef INS_MECHANIZATION_FACTORY_H
#define INS_MECHANIZATION_FACTORY_H

#include <memory>
#include <string>
#include "INSMechanizationBase.h"
#include "TangentialFrameMechanization.h"

// Factory for creating INS mechanization objects
class INSMechanizationFactory {
public:
    static std::shared_ptr<INSMechanizationBase> create(const std::string& type,
                                                        const State& init_state,
                                                        const IMUSpec& imu_spec) {
        if (type == "Tangential") {
            return std::make_shared<TangentialFrameMechanization>(init_state, imu_spec);
        }
        // Add other frame types here (e.g., NED, ECEF)
        return nullptr; // or throw std::invalid_argument
    }
};

#endif // INS_MECHANIZATION_FACTORY_H