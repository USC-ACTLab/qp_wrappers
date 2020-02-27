#ifndef QPWRAPPERS_ALGLIB_HPP
#define QPWRAPPERS_ALGLIB_HPP

#include "types.hpp"
#include "problem.hpp"
#include <ap.h>
#include <optimization.h>
#include <iostream>
#include <limits>

namespace qp_wrappers {
namespace alglib {  
  class solver {
  public:
    solver() {}

    return_type solve(const qp<double>& problem, typename qp<double>::Vector& primal_solution) {

      ::alglib::real_2d_array q, a;
      // q.setcontent(problem.Q().data());
      // a.setcontent(problem.A().data());
      q.attach_to_ptr(problem.num_vars(), problem.num_vars(), const_cast<double*>(problem.Q().data()));

      // std::cout.precision(std::numeric_limits<double>::max_digits10);
      // for(int i = 0; i < problem.num_vars(); i++) {
      //   for(int j = i; j < problem.num_vars(); j++) {
      //     if(problem.Q()(i,j) != problem.Q()(j, i)) {
      //       std::cout << i << " " << j << " " << problem.Q()(i, j) << " " << problem.Q()(j, i) << std::endl;
      //     }
      //   }
      // }

      // std::cout << q.rows() << " " << q.cols() << std::endl;
      // std::cout << problem.num_vars() << std::endl;

      ::alglib::real_1d_array lbx, ubx, lb, ub, c;
      // lbx.setcontent(problem.lbx().data());
      // ubx.setcontent(problem.ubx().data());
      // lb.setcontent(problem.lb().data());
      // ub.setcontent(problem.ub().data());
      // c.setcontent(problem.c().data());

      lbx.attach_to_ptr(problem.num_vars(), const_cast<double*>(problem.lbx().data()));
      ubx.attach_to_ptr(problem.num_vars(), const_cast<double*>(problem.ubx().data()));
      c.attach_to_ptr(problem.num_vars(), const_cast<double*>(problem.c().data()));

      ::alglib::minqpstate state;
      ::alglib::minqpcreate(problem.num_vars(), state);

      // std::cout << "0.2" << std::endl;
      try {
      ::alglib::minqpsetquadraticterm(state, q, true);
      } catch(::alglib::ap_error& err) {
        std::cout << err.msg << std::endl;
        throw;
      }
      // std::cout << "0.3" << std::endl;
      ::alglib::minqpsetlinearterm(state, c);
      // std::cout << "0.4" << std::endl;
      ::alglib::minqpsetbc(state, lbx, ubx);
      // std::cout << "0.5" << std::endl;

      if(problem.num_constraints() > 0) {
        std::cout << "attaching constraints" << std::endl;
        // std::cout << "1" << std::endl;
        a.attach_to_ptr(problem.num_constraints(), problem.num_vars(), const_cast<double*>(problem.A().data()));
        // std::cout << "2" << std::endl;
        lb.attach_to_ptr(problem.num_constraints(), const_cast<double*>(problem.lb().data()));
        ub.attach_to_ptr(problem.num_constraints(), const_cast<double*>(problem.ub().data()));
        // std::cout << "3" << std::endl;
        ::alglib::minqpsetlc2dense(state, a, lb, ub, problem.num_constraints());
        // std::cout << "4" << std::endl;
      }

      // set algorithm
      ::alglib::minqpsetalgobleic(state, 0.0, 0.0, 1e-7, 0);
      
      // ::alglib::minqpsetalgosparseipm(state, 1e-7);

      try {
      ::alglib::minqpoptimize(state);
      } catch(::alglib::ap_error& msg) {
        std::cout << msg.msg << std::endl;
        throw;
      }
      ::alglib::real_1d_array x;
      ::alglib::minqpreport rep;
      ::alglib::minqpresults(state, x, rep);
      
      std::cout << "term type: " << rep.terminationtype << std::endl; 
      if(rep.terminationtype > 0) {
        primal_solution.resize(problem.num_vars());
        for(typename qp<double>::Index i = 0; i < problem.num_vars(); i++) {
          primal_solution(i) = x(i);
        }
        return success;
      } else {
        return infeasible;
      }
    }

  private:

  };
}
}

#endif