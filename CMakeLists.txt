cmake_minimum_required(VERSION 3.26)
project(task_relax)

set(CMAKE_CXX_STANDARD 14)

add_executable(task_relax main.cpp
        Vector_3d.h
        Vector_3d.cpp
        Distribution.h
        second_task/IntegralSpace.h
        first_task/Collision.h
        first_task/Collision.cpp
)
