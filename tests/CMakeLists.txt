cmake_minimum_required(VERSION 3.8)

set(This AmfitrackTests)


# Force GoogleTest to use the shared (DLL) runtime
option(gtest_force_shared_crt "Use shared (DLL) run-time lib even in Debug builds of gtest" ON)

# Add GoogleTest from the new location
add_subdirectory("../libraries/googletest" "googletest_build")

# 2) Now list your test sources
set(Sources
    AmfitrackTests.cpp
)

# 3) Create your test executable
add_executable(${This} ${Sources})

if (CMAKE_VERSION VERSION_GREATER 3.12)
    set_property(TARGET ${This} PROPERTY CXX_STANDARD 20)
endif()

# 3a) Specify additional include directories
target_include_directories(${This} PUBLIC
    "../src/Driver/"                       # <-- so we can do #include "Driver/ControllerDevice.hpp"
    "../libraries/openvr/headers"
)

# 4) Link gtest & driver_amfitrack
target_link_libraries(${This} PUBLIC
    gtest_main
    driver_amfitrack
)

# 5) Register the tests
add_test(
    NAME ${This}
    COMMAND ${This}
)
