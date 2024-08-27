cmake_minimum_required(VERSION 3.5)

project(tsocs_core)

find_package(Ceres REQUIRED)
find_package (Eigen3 3.3 REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/problem_solver.cpp
  src/random_problem_generator.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(simple_test
  src/TestMain.cpp
)

target_link_libraries(simple_test
  ${CERES_LIBRARIES}
)