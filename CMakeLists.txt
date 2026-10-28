cmake_minimum_required(VERSION 3.18)

project(HybridMPCPID LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)

FetchContent_Declare(
    osqp
    GIT_REPOSITORY https://github.com/osqp/osqp.git
    GIT_TAG v1.0.0
)
FetchContent_MakeAvailable(osqp)

add_subdirectory(src)
