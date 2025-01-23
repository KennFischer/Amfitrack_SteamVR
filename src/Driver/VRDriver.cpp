#include "VRDriver.hpp"
#include <Driver/HMDDevice.hpp>
#include <Driver/TrackerDevice.hpp>
#include <Driver/ControllerDevice.hpp>
#include "amfitrack_cpp_SDK/Amfitrack.hpp"

vr::EVRInitError AmfitrackDriver::VRDriver::Init(vr::IVRDriverContext* pDriverContext)
{
    // Perform driver context initialisation
    if (vr::EVRInitError init_error = vr::InitServerDriverContext(pDriverContext); init_error != vr::EVRInitError::VRInitError_None) {
        return init_error;
    }

    Log("Activating AmfitrackDriver...");

    // Create instance of Amfitrack
    AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
    // Initialize the USB and connect to the devices
    AMFITRACK.initialize_amfitrack();
    // Start the main thread that reads the data
    AMFITRACK.start_amfitrack_task();

    Log("Amfitrack started"); // Log Amfitrack started message

    // Add a HMD
    this->AddDevice(std::make_shared<TrackerDevice>(4, "Amfitrack_HMDTrackedDevice"));

    // Add a couple controllers
    this->AddDevice(std::make_shared<ControllerDevice>(2, "Amfitrack_Controller_Left", ControllerDevice::Handedness::LEFT));
    this->AddDevice(std::make_shared<ControllerDevice>(3, "Amfitrack_Controller_Right", ControllerDevice::Handedness::RIGHT));

    // Add a tracker
    // this->AddDevice(std::make_shared<TrackerDevice>("Example_TrackerDevice"));



    Log("AmfitrackDriver Loaded Successfully");

	return vr::VRInitError_None;
}

void AmfitrackDriver::VRDriver::Cleanup()
{
}

void AmfitrackDriver::VRDriver::RunFrame()
{
    // Collect events
    vr::VREvent_t event;
    std::vector<vr::VREvent_t> events;
    while (vr::VRServerDriverHost()->PollNextEvent(&event, sizeof(event)))
    {
        events.push_back(event);
    }
    this->openvr_events_ = events;

    // Update frame timing
    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
    this->frame_timing_ = std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_frame_time_);
    this->last_frame_time_ = now;

    // Update devices
    for (auto& device : this->devices_)
        device->Update();
}

bool AmfitrackDriver::VRDriver::ShouldBlockStandbyMode()
{
    return false;
}

void AmfitrackDriver::VRDriver::EnterStandby()
{
}

void AmfitrackDriver::VRDriver::LeaveStandby()
{
}

std::vector<std::shared_ptr<AmfitrackDriver::IVRDevice>> AmfitrackDriver::VRDriver::GetDevices()
{
    return this->devices_;
}

std::vector<vr::VREvent_t> AmfitrackDriver::VRDriver::GetOpenVREvents()
{
    return this->openvr_events_;
}

std::chrono::milliseconds AmfitrackDriver::VRDriver::GetLastFrameTime()
{
    return this->frame_timing_;
}

bool AmfitrackDriver::VRDriver::AddDevice(std::shared_ptr<IVRDevice> device)
{
    vr::ETrackedDeviceClass openvr_device_class;
    // Remember to update this switch when new device types are added
    switch (device->GetDeviceType()) {
        case DeviceType::CONTROLLER:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_Controller;
            break;
        case DeviceType::HMD:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_HMD;
            break;
        case DeviceType::TRACKER:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_GenericTracker;
            break;
        case DeviceType::TRACKING_REFERENCE:
            openvr_device_class = vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference;
            break;
        default:
            return false;
    }
    bool result = vr::VRServerDriverHost()->TrackedDeviceAdded(device->GetSerial().c_str(), openvr_device_class, device.get());
    if(result)
        this->devices_.push_back(device);
    return result;
}

AmfitrackDriver::SettingsValue AmfitrackDriver::VRDriver::GetSettingsValue(std::string key)
{
    vr::EVRSettingsError err = vr::EVRSettingsError::VRSettingsError_None;
    int int_value = vr::VRSettings()->GetInt32(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return int_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;
    float float_value = vr::VRSettings()->GetFloat(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return float_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;
    bool bool_value = vr::VRSettings()->GetBool(settings_key_.c_str(), key.c_str(), &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return bool_value;
    }
    std::string str_value;
    str_value.reserve(1024);
    vr::VRSettings()->GetString(settings_key_.c_str(), key.c_str(), str_value.data(), 1024, &err);
    if (err == vr::EVRSettingsError::VRSettingsError_None) {
        return str_value;
    }
    err = vr::EVRSettingsError::VRSettingsError_None;

    return SettingsValue();
}

void AmfitrackDriver::VRDriver::Log(std::string message)
{
    std::string message_endl = message + "\n";
    vr::VRDriverLog()->Log(message_endl.c_str());
}

vr::IVRDriverInput* AmfitrackDriver::VRDriver::GetInput()
{
    return vr::VRDriverInput();
}

vr::CVRPropertyHelpers* AmfitrackDriver::VRDriver::GetProperties()
{
    return vr::VRProperties();
}

vr::IVRServerDriverHost* AmfitrackDriver::VRDriver::GetDriverHost()
{
    return vr::VRServerDriverHost();
}
