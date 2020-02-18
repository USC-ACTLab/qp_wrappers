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

# To run gurobi:

After installing gurobi, set the path set correctly as follows:
```
export GUROBI_INCLUDE_DIRS=<path to include directory in gurobi library> 
export GUROBI_LIBRARIES=<path to lib directory in gurobi library> 

In Mac for example:
export GUROBI_INCLUDE_DIRS=/Library/gurobi901/mac64/include
export GUROBI_LIBRARIES=/Library/gurobi901/mac64/lib
```
After this, modify the paths used in CMakeLists.txt in target_link_libraries function to link -lgurobi_c++ and -lgurobi90.(Use the same path as above)
Then Run the commands which is used to compile and run the code for linear regression and general solver.