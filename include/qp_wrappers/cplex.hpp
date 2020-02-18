#ifndef QPWRAPPERS_CPLEX_HPP
#define QPWRAPPERS_CPLEX_HPP

#include "problem.hpp"
#include "types.hpp"
#include <ilcplex/ilocplex.h>
#include <iostream>

namespace qp_wrappers {
    namespace cplex {
        
        class solver {
            public:

            template <typename T>
            return_type solve(const qp<T>& problem, typename qp<T>::Vector& primal_solution) {
                IloEnv env;
                IloModel model(env);

                IloNumVarArray variables(env);
                for(int i = 0; i < problem.variable_count; i++) {
                    IloNumVar var(env, problem.lbx(i), problem.ubx(i), ILOFLOAT);
                    variables.add(var);
                }
                for(int i = 0; i < problem.A.rows(); i++) {
                    IloExpr expr(env);
                    for(int j = 0; j < problem.variable_count; j++) {
                        expr += problem.A(i, j) * variables[j];
                    }
                    IloRange range(env, problem.lb(i), expr, problem.ub(i));
                    model.add(range);
                }

                IloExpr quadratic_cost(env);
                IloExpr linear_cost(env);
                for(int i = 0; i < problem.Q.rows(); i++) {
                    for(int j = 0; j < problem.Q.cols(); j++) {
                        quadratic_cost += variables[i] * variables[j] * problem.Q(i, j) * 0.5;
                    }
                    linear_cost += variables[i] * problem.c(i);
                }


                IloObjective obj(env, quadratic_cost + linear_cost, IloObjective::Minimize);
                model.add(obj);

                IloCplex cplex(model);
                cplex.setOut(env.getNullStream());
                bool solved = cplex.solve();

                if(solved) {
                    IloNumArray sol(env);
                    cplex.getValues(sol, variables);
                    primal_solution.resize(problem.variable_count);
                    for(int i = 0; i < problem.variable_count; i++) {
                        primal_solution(i) = sol[i];
                    }
                    env.end();
                    return success;
                } else {
                    env.end();
                    return infeasible;
                }
            }

        };

    }
}

#endif