#ifndef QPWRAPPERS_GUROBI_HPP
#define QPWRAPPERS_GUROBI_HPP

#include "problem.hpp"
#include "types.hpp"
#include <gurobi_c++.h>
#include <string>

namespace qp_wrappers{

	namespace gurobi {
		class solver{
			public:

				GRBEnv env;
				
				solver(){
					env.set("OutputFlag", "0");
				}


				template<typename T>
				return_type solve(const qp<T>& problem, typename qp<T>::Vector& primal_solution) {
					
					env.start();
					// Create an empty model
					GRBModel model = GRBModel(env);

					int n = problem.variable_count, m = problem.A.rows();
					
					GRBVar *x = model.addVars(problem.lbx.data(), problem.ubx.data(), NULL, NULL, NULL, n);

					for(int i=0;i<m;i++){
						GRBLinExpr lhs = 0;
						for(int j=0;j<n;j++){
							lhs += problem.A(i, j)*x[j];
						}
						model.addConstr(lhs <= problem.ub(i));
						model.addConstr(lhs >= problem.lb(i));
					}


					GRBQuadExpr obj1 = 0;

					for(int j=0;j<n;j++){
						obj1 += problem.c(j)*x[j];
					}

					GRBQuadExpr obj2 = 0;
					for(int i=0;i<n;i++){
						for(int j=0;j<n;j++){
							obj2 += problem.Q(i, j)*x[i]*x[j];
						}
					}
					obj2 *= 0.5;
					GRBQuadExpr obj = obj1 + obj2;

					model.setObjective(obj);
					model.optimize();

					if (model.get(GRB_IntAttr_Status) == GRB_OPTIMAL) {

						primal_solution.resize(problem.variable_count);
						
						for (int i = 0; i < n; i++)
							primal_solution(i) = x[i].get(GRB_DoubleAttr_X);
						
						delete[] x;
						
						return success;
					}

					delete[] x;
					
					return infeasible;

				}
		};
	}
}

#endif
