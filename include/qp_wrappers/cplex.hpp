#ifndef QPWRAPPERS_CPLEX_HPP
#define QPWRAPPERS_CPLEX_HPP

#include "problem.hpp"
#include "types.hpp"
#include <ilcplex/ilocplex.h>
#include <iostream>

namespace QPWrappers {
    namespace CPLEX {
        
        /*
            A QP engine that solves consecutive QP instances where the result of the previous
            instance used as an initial guess to the next one unless an initial guess
            is provided.
        */
        template<typename T>
        class Engine {
            public:
                Engine(): feasibility_tolerance(1e-6)/*, psd_tolerance(0)*/ {
                }

                Engine(const Engine& rhs) = delete;
                Engine& operator=(const Engine& rhs) = delete;

                Engine(Engine&& rhs) = delete;
                Engine& operator=(Engine&& rhs) = delete;

                ~Engine() {
                }

                /*
                    By how much are the constraints allowed to be violated?
                */
                void setFeasibilityTolerance(T tolerance) {
                    feasibility_tolerance = tolerance;
                }

                // /*
                //     By how much are the eigenvalues of the Q matrix are allowed be below 0 during PSD check?
                // */
                // void setPSDCheckEigenvalueTolerance(T tolerance) {
                //     psd_tolerance = tolerance;
                // }

                /*
                    Solve the first intance of the set of problems.
                    Load solution result to result.
                */
                OptReturnType init(const Problem<T>& problem, typename Problem<T>::Vector& result) {
                    IloEnv env;
                    env.setOut(env.getNullStream());

                    IloModel model(env);
                    IloNumVarArray variables(env);
                    for(int i = 0; i < problem.num_vars(); i++) {
                        IloNumVar var(env, problem.lbx()(i), problem.ubx()(i), ILOFLOAT);
                        variables.add(var);
                    }

                    for(int i = 0; i < problem.num_constraints(); i++) {
                        IloExpr expr(env);
                        for(int j = 0; j < problem.num_vars(); j++) {
                            expr += problem.A()(i, j) * variables[j];
                        }
                        IloRange range(env, problem.lb()(i), expr, problem.ub()(i));
                        model.add(range);
                    }

                    IloExpr quadratic_cost(env);
                    IloExpr linear_cost(env);
                    for(int i = 0; i < problem.num_vars(); i++) {
                        for(int j = 0; j < problem.num_vars(); j++) {
                            quadratic_cost += variables[i] * variables[j] * problem.Q()(i, j) * 0.5;
                        }
                        linear_cost += variables[i] * problem.c()(i);
                    }


                    IloObjective obj(env, quadratic_cost + linear_cost, IloObjective::Minimize);
                    model.add(obj);

                    IloCplex cplex(model);
                    cplex.setOut(env.getNullStream());
                    cplex.setWarning(env.getNullStream());
                    if(problem.is_Q_psd()) {
                        cplex.setParam(IloCplex::Param::Simplex::Tolerances::Feasibility, feasibility_tolerance);
                        cplex.setParam(IloCplex::Param::RootAlgorithm, IloCplex::Primal);
                        cplex.setParam(IloCplex::Param::OptimalityTarget, CPX_OPTIMALITYTARGET_OPTIMALCONVEX);
                    } else {
                        cplex.setParam(IloCplex::Param::OptimalityTarget, CPX_OPTIMALITYTARGET_FIRSTORDER);
                    }

                    cplex.solve();

                    auto status = cplex.getStatus();

                    if(status == IloAlgorithm::Status::Unknown) {
                        env.end();
                        return OptReturnType::Unknown;
                    } else if(status == IloAlgorithm::Status::Feasible) {
                        loadResult(env, cplex, variables, result);
                        env.end();
                        return OptReturnType::Feasible;
                    } else if(status == IloAlgorithm::Status::Optimal) {
                        loadResult(env, cplex, variables, result);
                        env.end();
                        return OptReturnType::Optimal;
                    } else if(status == IloAlgorithm::Status::Infeasible) {
                        env.end();
                        return OptReturnType::Infeasible;
                    } else if(status == IloAlgorithm::Status::Unbounded) {
                        loadResult(env, cplex, variables, result);
                        env.end();
                        return OptReturnType::Unbounded;
                    } else if(status == IloAlgorithm::Status::InfeasibleOrUnbounded) {
                        env.end();
                        return OptReturnType::InfeasibleOrUnbounded;
                    } else if(status == IloAlgorithm::Status::Error) {
                        env.end();
                        return OptReturnType::Error;
                    }

                    env.end();
                    return OptReturnType::Unknown;
                }

                /*
                    Solve the next problem of the set of problems.
                */
                OptReturnType next(const Problem<T>& problem, typename Problem<T>::Vector& result) {
                    return init(problem, result);
                }

                /*
                    Solve the next problem of the set of problems with the given initial guess.
                    Since there is no concept of feeding initial guess in CPLEX, we simply call init.
                */
                OptReturnType next(const Problem<T>& problem, typename Problem<T>::Vector& result, const typename Problem<T>::Vector& initial_guess) {
                    return init(problem, result);
                }

            private:

                T feasibility_tolerance;
                // T psd_tolerance;

                /*
                    Load solution result to the result.
                */
                void loadResult(const IloEnv& env, const IloCplex& cplex, const IloNumVarArray& variables, typename Problem<T>::Vector& result) {
                    IloNumArray sol(env);
                    cplex.getValues(sol, variables);
                    result.resize(variables.getSize());
                    for(typename Problem<T>::Index i = 0; i < result.rows(); i++) {
                        result(i) = sol[i];
                    }
                }


        };
    }
}

#endif