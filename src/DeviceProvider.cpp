#include <DeviceProvider.h>
#include "amfitrack_cpp_SDK/Amfitrack.hpp"

EVRInitError DeviceProvider::Init(IVRDriverContext* pDriverContext)
{
    EVRInitError initError = InitServerDriverContext(pDriverContext);
    if (initError != EVRInitError::VRInitError_None)
    {
        return initError;
    }
    
    VRDriverLog()->Log("Initializing Amfitrack controller"); //this is how you log out Steam's log file.

    /* Create instance of amfitrack */
    AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
    /* Initializes the USB and connects to the devices */
    AMFITRACK.initialize_amfitrack();
    /* Starts the main thread, that read the data */
    AMFITRACK.start_amfitrack_task();

    VRDriverLog()->Log("Amfitrack started"); //this is how you log out Steam's log file.

    controllerDriverLeft = new ControllerDriver(2, ControllerDriver::Handedness::LEFT);
    VRServerDriverHost()->TrackedDeviceAdded("Amfitrack_controller_left", TrackedDeviceClass_Controller, controllerDriverLeft); //add all your devices like this.

    controllerDriverRight = new ControllerDriver(3, ControllerDriver::Handedness::RIGHT);
    VRServerDriverHost()->TrackedDeviceAdded("Amfitrack_controller_right", TrackedDeviceClass_Controller, controllerDriverRight); //add all your devices like this.

    VRDriverLog()->Log("Initializing completed!"); //this is how you log out Steam's log file.

    return vr::VRInitError_None;
}

void DeviceProvider::Cleanup()
{
    delete controllerDriverLeft;
    delete controllerDriverRight;
    controllerDriverLeft = NULL;
    controllerDriverRight = NULL;
}
const char* const* DeviceProvider::GetInterfaceVersions()
{
    return k_InterfaceVersions;
}

void DeviceProvider::RunFrame()
{
    controllerDriverLeft->RunFrame();
    controllerDriverRight->RunFrame();
}

bool DeviceProvider::ShouldBlockStandbyMode()
{
    return false;
}

void DeviceProvider::EnterStandby() {}

void DeviceProvider::LeaveStandby() {}