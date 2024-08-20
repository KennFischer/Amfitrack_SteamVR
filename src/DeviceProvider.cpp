#include <DeviceProvider.h>
#include "amfitrack_cpp_SDK/Amfitrack.hpp"



// Initializes the device provider
EVRInitError DeviceProvider::Init(IVRDriverContext* pDriverContext)
{
    // Initialize the server driver context
    EVRInitError initError = InitServerDriverContext(pDriverContext);
    if (initError != EVRInitError::VRInitError_None)
    {
        return initError; // Return error if initialization fails
    }
    
    VRDriverLog()->Log("Initializing Amfitrack controller"); // Log initialization message

    // Create instance of Amfitrack
    AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
    // Initialize the USB and connect to the devices
    AMFITRACK.initialize_amfitrack();
    // Start the main thread that reads the data
    AMFITRACK.start_amfitrack_task();

    VRDriverLog()->Log("Amfitrack started"); // Log Amfitrack started message

    // Initialize left controller driver
    controllerDriverLeft = new ControllerDriver(2, ControllerDriver::Handedness::LEFT);
    VRServerDriverHost()->TrackedDeviceAdded("Amfitrack_controller_left", TrackedDeviceClass_Controller, controllerDriverLeft); // Add left controller device

    // Initialize right controller driver
    controllerDriverRight = new ControllerDriver(3, ControllerDriver::Handedness::RIGHT);
    VRServerDriverHost()->TrackedDeviceAdded("Amfitrack_controller_right", TrackedDeviceClass_Controller, controllerDriverRight); // Add right controller device

    VRDriverLog()->Log("Initializing completed!"); // Log initialization completed message

    return vr::VRInitError_None; // Return no error
}

// Cleans up the device provider
void DeviceProvider::Cleanup()
{
    // Delete left and right controller drivers
    delete controllerDriverLeft;
    delete controllerDriverRight;
    controllerDriverLeft = NULL; // Set left controller driver to NULL
    controllerDriverRight = NULL; // Set right controller driver to NULL
}

// Returns the interface versions
const char* const* DeviceProvider::GetInterfaceVersions()
{
    return k_InterfaceVersions; // Return interface versions
}

// Runs a frame for the device provider
void DeviceProvider::RunFrame()
{
    // Run frame for left and right controller drivers
    controllerDriverLeft->RunFrame();
    controllerDriverRight->RunFrame();
}

// Indicates whether to block standby mode
bool DeviceProvider::ShouldBlockStandbyMode()
{
    return false; // Do not block standby mode
}

// Enters standby mode
void DeviceProvider::EnterStandby() {}

// Leaves standby mode
void DeviceProvider::LeaveStandby() {}