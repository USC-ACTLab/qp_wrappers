# requires
- boost
- gmp library


# to compile & run the example code for linear regression & run general solver

```
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make linear_regression
./linear_regression < ../example/linear_regression/data/ex1
<all solvers must give the same output>

make general_solver
./general_solver < ../example/general_qp_solver/data/ex1
<all solvers must give the same output>
```