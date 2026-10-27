cmake_minimum_required(VERSION 3.16)

include(FetchContent)

FetchContent_Declare(
    osqp
    GIT_REPOSITORY https://github.com/osqp/osqp.git
    GIT_TAG v1.0.0
)

FetchContent_MakeAvailable(osqp)

add_library(tank_model common/TankModel.cpp)
target_include_directories(tank_model PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(pid_controller pid/PIDController.cpp)
target_include_directories(pid_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_library(mpc_controller mpc/MPCController.cpp)
target_include_directories(mpc_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR} ${osqp_SOURCE_DIR}/include)

target_link_libraries(mpc_controller PUBLIC osqp)

add_library(hybrid_controller hybrid/HybridController.cpp)
target_include_directories(hybrid_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

target_link_libraries(hybrid_controller PUBLIC pid_controller mpc_controller)

add_library(benchmark_lib benchmark/Benchmark.cpp)
target_include_directories(benchmark_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(benchmark_lib PUBLIC tank_model pid_controller mpc_controller hybrid_controller)

add_executable(hybrid_control_app main.cpp)
target_include_directories(hybrid_control_app PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(hybrid_control_app PRIVATE tank_model pid_controller mpc_controller hybrid_controller benchmark_lib osqp)
