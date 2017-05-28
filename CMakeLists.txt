project(MPC)

cmake_minimum_required (VERSION 3.5)

# -g allows for gdb debugging
# turn on -03 for best performance
add_definitions(-std=c++11 -O3)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/build")

set(sources src/MPC.cpp src/main.cpp src/CoordinateTransform.cpp src/CoordinateTransform.h src/helpers.h src/helpers.cpp)

include_directories(src/Eigen-3.3)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

include_directories(/usr/local/include)
link_directories(/usr/local/lib)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

add_executable(mpc ${sources})

target_link_libraries(mpc coinblas coinlapack ipopt z ssl uv uWS)

