# CMakeList.txt : CMake project for Amfitrack_SteamVR_driver, include source and define
# project specific logic here.
#
cmake_minimum_required(VERSION 3.8)

enable_testing()

# Globally defines WINDOWS_IGNORE_PACKING_MISMATCH for all targets
# add_compile_definitions(WINDOWS_IGNORE_PACKING_MISMATCH)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project("driver_amfitrack")

if(MSVC)
  # Force all targets (in all subdirectories) to use the shared runtime:
  set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")
endif()

# Enable verbose makefile output for debugging
set(CMAKE_VERBOSE_MAKEFILE ON)

set(COPY_DIRECTLY_TO_STEAM ON)

# Define source files
set(SOURCES
    src/Driver/ControllerDevice.cpp
    src/Driver/HMDDevice.cpp
    src/Driver/TrackerDevice.cpp
    src/Driver/VRDriver.cpp
    src/Driver/PoseHelper.cpp
    src/Native/DriverFactory.cpp
)

# Define header files
set(HEADERS
    src/Driver/ControllerDevice.hpp
    src/Driver/DeviceType.hpp
    src/Driver/HMDDevice.hpp
    src/Driver/IVRDevice.hpp
    src/Driver/IVRDriver.hpp
    src/Driver/TrackerDevice.hpp
    src/Driver/VRDriver.hpp
    src/Driver/vrmath.h
    src/Driver/PoseHelper.hpp
    src/Native/DriverFactory.hpp

    libraries/openvr/headers/openvr.h
    libraries/openvr/headers/openvr_capi.h
    libraries/openvr/headers/openvr_driver.h

    libraries/linalg/linalg.h

    libraries/amfitrack_cpp_SDK/Amfitrack.hpp
)

# Add include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Native
    ${CMAKE_CURRENT_SOURCE_DIR}/libraries
    ${CMAKE_CURRENT_SOURCE_DIR}/libraries/linalg
    ${CMAKE_CURRENT_SOURCE_DIR}/libraries/openvr/headers
)

# Add the shared library BEFORE adding subdirectories
add_library(driver_amfitrack SHARED ${SOURCES} ${HEADERS})

# Link third-party libraries
target_link_libraries(driver_amfitrack PUBLIC
    Amfitrack_API
)

# Define export macro for DLL
target_compile_definitions(driver_amfitrack PRIVATE AMFITRACK_EXPORTS)

# Set C++ Standard
if (CMAKE_VERSION VERSION_GREATER 3.12)
  set_property(TARGET driver_amfitrack PROPERTY CXX_STANDARD 20)
endif()

# Determine architecture (x64 or x86)
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(ARCHITECTURE "x64")
else()
    set(ARCHITECTURE "x86")
endif()


# Set destination directory based on architecture
if(ARCHITECTURE STREQUAL "x64")
    set(DESTINATION_DIR "C:/Program Files (x86)/Steam/steamapps/common/SteamVR/drivers/Amfitrack_SteamVR/bin/win64")
    set(DESTINATION_DIR_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/release/Amfitrack_SteamVR/bin/win64")
    set(HIDAPI_DLL "${CMAKE_CURRENT_SOURCE_DIR}/libraries/amfitrack_cpp_SDK/lib/hidapi/x64/hidapi.dll")
else()
    message( FATAL_ERROR "There is no support for x86!" )
    set(DESTINATION_DIR "C:/Program Files (x86)/Steam/steamapps/common/SteamVR/drivers/Amfitrack_SteamVR/bin/win32")
    set(DESTINATION_DIR_RELEASE "${CMAKE_CURRENT_SOURCE_DIR}/release/Amfitrack_SteamVR/bin/win32")
    set(HIDAPI_DLL "${CMAKE_CURRENT_SOURCE_DIR}/libraries/amfitrack_cpp_SDK/lib/hidapi/x86/hidapi.dll")
endif()
set(SOURCE_FILE "${CMAKE_CURRENT_BINARY_DIR}/driver_amfitrack.dll")

# Custom command to delete the Amfitrack_SteamVR folder before building
add_custom_command(
    TARGET driver_amfitrack PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_SOURCE_DIR}/release/Amfitrack_SteamVR"
)

add_custom_command(TARGET driver_amfitrack POST_BUILD
    COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_SOURCE_DIR}/copy_to_release_folder.cmake"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SOURCE_FILE}" "${DESTINATION_DIR_RELEASE}/driver_amfitrack.dll"
    #COMMAND ${CMAKE_COMMAND} -E copy_if_different "${HIDAPI_DLL}" "${DESTINATION_DIR_RELEASE}/hidapi.dll"
)

if(COPY_DIRECTLY_TO_STEAM)
add_custom_command(
    TARGET driver_amfitrack PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove_directory "C:/Program Files (x86)/Steam/steamapps/common/SteamVR/drivers/Amfitrack_SteamVR"
)

add_custom_command(TARGET driver_amfitrack POST_BUILD
    COMMAND ${CMAKE_COMMAND} -P "${CMAKE_CURRENT_SOURCE_DIR}/copy_if_different_directory.cmake"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${SOURCE_FILE}" "${DESTINATION_DIR}/driver_amfitrack.dll"
    #COMMAND ${CMAKE_COMMAND} -E copy_if_different "${HIDAPI_DLL}" "${DESTINATION_DIR}/hidapi.dll"
)
endif()

# Add subdirectories AFTER defining the library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libraries/Amfitrack_Cpp_SDK)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/libraries/openvr)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)

