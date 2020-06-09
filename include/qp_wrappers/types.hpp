#ifndef QPWRAPPERS_TYPES_HPP
#define QPWRAPPERS_TYPES_HPP

#include <iostream>

namespace QPWrappers {

enum class OptReturnType {
    Optimal,
    Feasible,
    Unbounded,
    Infeasible,
    Error,
    Unknown,
    InfeasibleOrUnbounded
};

}

std::ostream& operator<<(std::ostream& os,
        QPWrappers::OptReturnType ret_type) {
    using namespace QPWrappers;
    switch(ret_type) {
        case OptReturnType::Optimal:
            os << "Optimal";
            break;
        case OptReturnType::Feasible:
            os << "Feasible";
            break;
        case OptReturnType::Unbounded:
            os << "Unbounded";
            break;
        case OptReturnType::Infeasible:
            os << "Infeasible";
            break;
        case OptReturnType::Error:
            os << "Error";
            break;
        case OptReturnType::Unknown:
            os << "Unknown";
            break;
        case OptReturnType::InfeasibleOrUnbounded:
            os << "InfeasibleOrUnbounded";
            break;
    }

    return os;
}

#endif