#ifndef QPWRAPPERS_TYPES_HPP
#define QPWRAPPERS_TYPES_HPP

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

#endif