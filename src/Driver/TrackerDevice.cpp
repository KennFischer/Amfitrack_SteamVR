#include "TrackerDevice.hpp"
#include <Windows.h>
#include "amfitrack_cpp_SDK/Amfitrack.hpp"
#include "vrmath.h"

AmfitrackDriver::TrackerDevice::TrackerDevice(uint8_t deviceId, std::string serial) : deviceID_(deviceId),
                                                                                      serial_(serial)
{
}

std::string AmfitrackDriver::TrackerDevice::GetSerial()
{
    return this->serial_;
}

vr::EVRInitError AmfitrackDriver::TrackerDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    GetDriver()->Log("Activating tracker " + this->serial_);

    // Get the properties handle
    auto props = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);

    // Set some universe ID (Must be 2 or higher)
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_CurrentUniverseId_Uint64, 2);

    // Set up a model "number" (not needed but good to have)
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "Amfitrack_tracker");

    // Set up a render model path
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");

    // Opt out of hand selection
    GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_OptOut);

    return vr::EVRInitError::VRInitError_None;
}

void AmfitrackDriver::TrackerDevice::Update()
{
    // Check if the device is valid
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

    // Get the HMD's pose in SteamVR
    vr::TrackedDevicePose_t hmd_pose{};
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.f, &hmd_pose, 1); // Fetch the HMD pose

    // Check if the HMD pose is valid
    if (hmd_pose.eTrackingResult == vr::TrackingResult_Running_OK)
    {
        // Fetch the HMD position and orientation
        vr::HmdVector3_t hmd_position = HmdVector3_From34Matrix(hmd_pose.mDeviceToAbsoluteTracking);
        vr::HmdQuaternion_t hmd_orientation = HmdQuaternion_FromMatrix(hmd_pose.mDeviceToAbsoluteTracking);

        // Create a default pose for the tracker
        auto pose = IVRDevice::MakeDefaultPose();

        // Define the offset (5 cm forward and 12 cm up)
        vr::HmdVector3_t offset_position = {0.05f, 0.12f, 0.0f}; // X = 5 cm forward, Y = 12 cm above, Z = 0

        // Rotate the offset by the HMD's orientation
        vr::HmdVector3_t rotated_offset = RotateVectorByQuaternion(offset_position, hmd_orientation);

        // Calculate the tracker's position in SteamVR space
        pose.vecPosition[0] = hmd_position.v[0] + rotated_offset.v[0]; // X position
        pose.vecPosition[1] = hmd_position.v[1] + rotated_offset.v[1]; // Y position
        pose.vecPosition[2] = hmd_position.v[2] + rotated_offset.v[2]; // Z position

        // Set the tracker's rotation to match the HMD's rotation
        pose.qRotation = hmd_orientation;

        // Mark the pose as valid
        pose.poseIsValid = true;
        pose.result = vr::TrackingResult_Running_OK;

        // Update in SteamVR
        GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));

        // Store the last pose for reference
        this->last_pose_ = pose;
    }
    else
    {
        // If HMD pose is invalid, mark the tracker pose as invalid
        auto pose = IVRDevice::MakeDefaultPose();
        pose.poseIsValid = false;
        GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));

        // // Log invalid state
        // std::string log_message = "TrackerDevice::Update - Device ID: " + std::to_string(this->device_index_) +
        //                           ", Serial: " + this->GetSerial() +
        //                           ", Pose is invalid.";
        // GetDriver()->Log(log_message);
    }
}

DeviceType AmfitrackDriver::TrackerDevice::GetDeviceType()
{
    return DeviceType::TRACKER;
}

vr::TrackedDeviceIndex_t AmfitrackDriver::TrackerDevice::GetDeviceIndex()
{
    return this->device_index_;
}

void AmfitrackDriver::TrackerDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void AmfitrackDriver::TrackerDevice::EnterStandby()
{
}

void *AmfitrackDriver::TrackerDevice::GetComponent(const char *pchComponentNameAndVersion)
{
    return nullptr;
}

void AmfitrackDriver::TrackerDevice::DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t AmfitrackDriver::TrackerDevice::GetPose()
{
    return last_pose_;
}