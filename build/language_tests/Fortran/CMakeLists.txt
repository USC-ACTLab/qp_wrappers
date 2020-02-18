project(test NONE)
    cmake_minimum_required(VERSION 2.8.11)
    set (CMAKE_Fortran_FLAGS "")
    set (CMAKE_EXE_LINKER_FLAGS "-L/usr/local/opt/libffi/lib")
    enable_language(Fortran)
  