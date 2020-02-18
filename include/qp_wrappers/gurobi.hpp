#include "problem.hpp"
#include "types.hpp"
#include <gurobi_c++.h>
#include <string>

namespace qp_wrappers{

	namespace gurobi {
		class solver{
			public:
				solver(){

				}


				template<typename T>
				return_type solve(const qp<T>& problem, typename qp<T>::Vector& primal_solution) {

					GRBEnv env = GRBEnv(true);
					//env.set("LogFile", "mip1.log");
					env.start();

					// Create an empty model
					GRBModel model = GRBModel(env);

					qp<c_float>::Matrix A = problem.A;
					qp<c_float>::Matrix Q = problem.Q;

					int n = problem.variable_count, m = problem.A.rows();
					
					qp<c_float>::Vector lb = problem.lb;
					qp<c_float>::Vector ub = problem.ub;
					qp<c_float>::Vector lbx = problem.lbx;
					qp<c_float>::Vector ubx = problem.ubx;
					qp<c_float>::Vector c = problem.c;

					double * lB = new double[n];
					for(int i=0;i<n;i++)	lB[i] = lbx(i);
					double * uB = new double[m];
					for(int i=0;i<n;i++)	uB[i] = ubx(i);

					double * q = new double[n*n];
					int index = 0;
					for(int i=0;i<n;i++){
						for(int j=0;j<n;j++){
							q[index] = Q(i, j);
							index++;
						}
					}

					double * a = new double[m*n];
					index = 0;
					for(int i=0;i<m;i++){
						for(int j=0;j<n;j++){
							a[index] = A(i, j);
							index++;
						}
					}

					GRBVar *x = model.addVars(lB, uB, NULL, NULL, NULL, n);

					for(int i=0;i<m;i++){
						GRBLinExpr lhs = 0;
						for(int j=0;j<n;j++){
							lhs += a[i*n+j]*x[j];
						}
						model.addConstr(lhs <= ub(i));
						model.addConstr(lhs >= lb(i));
					}


					GRBQuadExpr obj1 = 0;

					for(int j=0;j<n;j++){
						obj1 += c(j)*x[j];
					}

					GRBQuadExpr obj2 = 0;
					for(int i=0;i<n;i++){
						for(int j=0;j<n;j++){
							obj2 += q[i*n+j]*x[i]*x[j];
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
						delete[] a;
						delete[] q;
						delete[] lB;
						delete[] uB;
						return success;
					}

					delete[] x;
					delete[] a;
					delete[] q;
					delete[] lB;
					delete[] uB;

					return infeasible;

				}
		};
	}
}
