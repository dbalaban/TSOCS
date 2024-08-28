cmake_minimum_required(VERSION 3.12)
project(TSOCS_top)

set(CMAKE_CXX_STANDARD 17)

# install to bin directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Add the "src" directory
add_subdirectory(src)