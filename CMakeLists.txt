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
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src/Native)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/linalg)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries/openvr/headers)


add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libraries/Amfitrack_Cpp_SDK)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libraries/openvr)

# Source files
set(SOURCES
    src/Driver/ControllerDevice.cpp
    src/Driver/HMDDevice.cpp
    src/Driver/TrackerDevice.cpp
    src/Driver/TrackingReferenceDevice.cpp
    src/Driver/VRDriver.cpp
    src/Native/DriverFactory.cpp
)

# Header files
set(HEADERS
    src/Driver/ControllerDevice.hpp
    src/Driver/DeviceType.hpp
    src/Driver/HMDDevice.hpp
    src/Driver/IVRDevice.hpp
    src/Driver/IVRDriver.hpp
    src/Driver/TrackerDevice.hpp
    src/Driver/TrackingReferenceDevice.hpp
    src/Driver/VRDriver.hpp
    src/Native/DriverFactory.hpp

    libraries/openvr/headers/openvr.h
    libraries/openvr/headers/openvr_capi.h
    libraries/openvr/headers/openvr_driver.h

    libraries/linalg/linalg.h

    libraries/amfitrack_cpp_SDK/Amfitrack.hpp
)

# Add source to this project's executable.
add_library(driver_amfitrack SHARED ${SOURCES} ${HEADERS})

target_link_libraries(driver_amfitrack Amfitrack_API)

if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET driver_amfitrack PROPERTY CXX_STANDARD 20)
endif()

set(SOURCE_FILE "${CMAKE_CURRENT_BINARY_DIR}/driver_amfitrack.dll")
set(DESTINATION_DIR "C:/Program Files (x86)/Steam/steamapps/common/SteamVR/drivers/Amfitrack_SteamVR/bin/win64")

add_custom_command(TARGET driver_amfitrack POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy "${SOURCE_FILE}" "${DESTINATION_DIR}/driver_amfitrack.dll"
    COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_SOURCE_DIR}/copy_if_different_directory.cmake"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${CMAKE_CURRENT_SOURCE_DIR}/driver/amfitrack/driver.vrdrivermanifest" "C:/Program Files (x86)/Steam/steamapps/common/SteamVR/drivers/Amfitrack_SteamVR/"
)
 

# TODO: Add tests and install targets if needed.
