# requires
- boost


# to compile & run the example code for linear regression

```
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make linear_regression
./linear_regression < ../example/linear_regression/data/ex1
<all solvers must give the same output>
```