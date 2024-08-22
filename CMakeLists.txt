# CMakeList.txt : CMake project for Amfitrack_SteamVR_driver, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.8)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project ("driver_amfitrack")

# Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/driver_files/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/driver_files/src/Native)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/linalg)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/openvr/headers)


add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libraries/Amfitrack_Cpp_SDK)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libraries/openvr)

# Source files
set(SOURCES
    driver_files/src/Driver/ControllerDevice.cpp
    driver_files/src/Driver/HMDDevice.cpp
    driver_files/src/Driver/TrackerDevice.cpp
    driver_files/src/Driver/TrackingReferenceDevice.cpp
    driver_files/src/Driver/VRDriver.cpp
    driver_files/src/Native/DriverFactory.cpp
)

# Header files
set(HEADERS
    driver_files/src/Driver/ControllerDevice.hpp
    driver_files/src/Driver/DeviceType.hpp
    driver_files/src/Driver/HMDDevice.hpp
    driver_files/src/Driver/IVRDevice.hpp
    driver_files/src/Driver/IVRDriver.hpp
    driver_files/src/Driver/TrackerDevice.hpp
    driver_files/src/Driver/TrackingReferenceDevice.hpp
    driver_files/src/Driver/VRDriver.hpp
    driver_files/src/Native/DriverFactory.hpp

    libraries/openvr/headers/openvr.h
    libraries/openvr/headers/openvr_capi.h
    libraries/openvr/headers/openvr_driver.h

    libraries/linalg/linalg.h

    libraries/amfitrack_cpp_SDK/Amfitrack.hpp
)

# Add source to this project's executable.
add_library (driver_amfitrack SHARED ${SOURCES} ${HEADERS})

target_link_libraries(driver_amfitrack Amfitrack_API)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET driver_amfitrack PROPERTY CXX_STANDARD 20)
endif()

set(SOURCE_FILE "out/build/x64-debug/driver_amfitrack.dll")
set(DESTINATION_DIR "C:/Users/Domin/Desktop")

add_custom_command(TARGET driver_amfitrack POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/driver_amfitrack.dll" "C:/Program Files (x86)/Steam/steamapps/common/SteamVR/drivers/Amfitrack_SteamVR/bin/win64/driver_amfitrack.dll"
)
 

# TODO: Add tests and install targets if needed.
