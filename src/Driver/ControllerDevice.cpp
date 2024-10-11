#include "ControllerDevice.hpp"
#include <Windows.h>
#include "amfitrack_cpp_SDK/Amfitrack.hpp"
#include "vrmath.h"
#include "TrackerDevice.hpp"

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

AmfitrackDriver::ControllerDevice::ControllerDevice(uint8_t deviceId, std::string serial, ControllerDevice::Handedness handedness) : deviceID_(deviceId),
                                                                                                                                     serial_(serial),
                                                                                                                                     handedness_(handedness)
{
}

std::string AmfitrackDriver::ControllerDevice::GetSerial()
{
    return this->serial_;
}

vr::EVRInitError AmfitrackDriver::ControllerDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    GetDriver()->Log("Activating controller " + this->serial_);

    // Get the properties handle
    this->props_ = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);

    // Set some universe ID (Must be 2 or higher)
    GetDriver()->GetProperties()->SetUint64Property(this->props_, vr::Prop_CurrentUniverseId_Uint64, 2);

    // Set up a model "number" (not needed but good to have)
    GetDriver()->GetProperties()->SetStringProperty(this->props_, vr::Prop_ModelNumber_String, "amfitrack_controller");

    // Set up a render model path
    GetDriver()->GetProperties()->SetStringProperty(this->props_, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");

    // Setup inputs and outputs
    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/a/click", &this->a_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/a/touch", &this->a_button_touch_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/b/click", &this->b_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/b/touch", &this->b_button_touch_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/trigger/click", &this->trigger_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/trigger/touch", &this->trigger_touch_component_);
    GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/trigger/value", &this->trigger_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

    // GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/grip/touch", &this->grip_touch_component_);
    // GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/grip/value", &this->grip_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
    // GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/grip/force", &this->grip_force_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/system/click", &this->system_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/system/touch", &this->system_touch_component_);

    // GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/trackpad/click", &this->trackpad_click_component_);
    // GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/trackpad/touch", &this->trackpad_touch_component_);
    // GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/trackpad/x", &this->trackpad_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    // GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/trackpad/y", &this->trackpad_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);

    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/joystick/click", &this->joystick_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/joystick/touch", &this->joystick_touch_component_);
    GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/joystick/x", &this->joystick_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/joystick/y", &this->joystick_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);

    // Give SteamVR a hint at what hand this controller is for
    if (this->handedness_ == Handedness::LEFT)
    {
        GetDriver()->GetProperties()->SetInt32Property(this->props_, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_LeftHand);
    }
    else if (this->handedness_ == Handedness::RIGHT)
    {
        GetDriver()->GetProperties()->SetInt32Property(this->props_, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_RightHand);
    }
    else
    {
        GetDriver()->GetProperties()->SetInt32Property(this->props_, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_OptOut);
    }

    // Set controller profile
    GetDriver()->GetProperties()->SetStringProperty(this->props_, vr::Prop_InputProfilePath_String, "{amfitrack}/input/amfitrack_controller_profile.json");

    // std::string controller_ready_file = "{amfitrack}/icons/controller_status_ready.png";
    // std::string controller_not_ready_file = "{amfitrack}/icons/controller_off.png";

    // GetDriver()->GetProperties()->SetStringProperty(this->props_, vr::Prop_NamedIconPathDeviceReady_String, controller_ready_file.c_str());

    // GetDriver()->GetProperties()->SetStringProperty(this->props_, vr::Prop_NamedIconPathDeviceOff_String, controller_not_ready_file.c_str());
    // GetDriver()->GetProperties()->SetStringProperty(this->props_, vr::Prop_NamedIconPathDeviceSearching_String, controller_not_ready_file.c_str());
    // GetDriver()->GetProperties()->SetStringProperty(this->props_, vr::Prop_NamedIconPathDeviceSearchingAlert_String, controller_not_ready_file.c_str());
    // GetDriver()->GetProperties()->SetStringProperty(this->props_, vr::Prop_NamedIconPathDeviceReadyAlert_String, controller_not_ready_file.c_str());
    // GetDriver()->GetProperties()->SetStringProperty(this->props_, vr::Prop_NamedIconPathDeviceNotReady_String, controller_not_ready_file.c_str());
    // GetDriver()->GetProperties()->SetStringProperty(this->props_, vr::Prop_NamedIconPathDeviceStandby_String, controller_not_ready_file.c_str());
    // GetDriver()->GetProperties()->SetStringProperty(this->props_, vr::Prop_NamedIconPathDeviceAlertLow_String, controller_not_ready_file.c_str());

    return vr::VRInitError_None;
}

// void AmfitrackDriver::ControllerDevice::Update()
// {
//     // Ensure the device index is valid
//     if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
//         return;

//     // Initialize the default pose for the controller
//     auto pose = IVRDevice::MakeDefaultPose();

//     // Get the Amfitrack system instance
//     AMFITRACK &AMFITRACK = AMFITRACK::getInstance();

//     // Check if the device is active
//     if (!AMFITRACK.getDeviceActive(this->deviceID_))
//     {
//         pose.poseIsValid = false; // Mark pose as invalid
//         GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
//         return;  // Exit if the device is not active
//     }

//     // Get the raw tracker pose from the Amfitrack system
//     lib_AmfiProt_Amfitrack_Pose_t tracker_raw_pose;
//     AMFITRACK.getDevicePose(4, &tracker_raw_pose); // Assuming device ID 4 is the tracker

//     // Retrieve the source pose in SteamVR coordinates
//     vr::DriverPose_t source_pose = GetSourcePose(tracker_raw_pose); // Pass the tracker_raw_pose

//     // Get the controller's raw pose from the Amfitrack system
//     lib_AmfiProt_Amfitrack_Pose_t controller_raw_pose;
//     AMFITRACK.getDevicePose(this->deviceID_, &controller_raw_pose); // Local controller pose

//     // Convert the raw controller pose's position to a vector
//     vr::HmdVector3_t controller_position_local = {
//         controller_raw_pose.position_x_in_m,
//         controller_raw_pose.position_y_in_m,
//         controller_raw_pose.position_z_in_m
//     };

//     // Calculate the offset of the controller relative to the source in Amfitrack space
//     vr::HmdVector3_t offset_position = {
//         controller_position_local.v[0], 
//         controller_position_local.v[1], 
//         controller_position_local.v[2]
//     };

//     // Transform the offset position by the source's orientation to get its position in SteamVR space
//     vr::HmdVector3_t transformed_offset = RotateVectorByQuaternion(offset_position, source_pose.qRotation);

//     // Calculate the final position of the controller in SteamVR space
//     pose.vecPosition[0] = source_pose.vecPosition[0] + transformed_offset.v[0]; // X position
//     pose.vecPosition[1] = source_pose.vecPosition[1] + transformed_offset.v[1]; // Y position
//     pose.vecPosition[2] = source_pose.vecPosition[2] + transformed_offset.v[2]; // Z position

//     // Handle the orientation of the controller
//     vr::HmdQuaternion_t controller_orientation_local = {
//         controller_raw_pose.orientation_w,
//         controller_raw_pose.orientation_x,
//         controller_raw_pose.orientation_y,
//         controller_raw_pose.orientation_z
//     };

//     // Combine the source orientation with the controller's local orientation
//     pose.qRotation = MultiplyQuaternions(source_pose.qRotation, controller_orientation_local);

//     // Normalize the quaternion to ensure valid rotation
//     pose.qRotation = HmdQuaternion_Normalize(pose.qRotation);

//     // Handle button press states
//     lib_AmfiProt_Amfitrack_Sensor_Measurement_t sensorMeasurement;
//     AMFITRACK.getSensorMeasurements(this->deviceID_, &sensorMeasurement);
//     uint16_t gpio_state = sensorMeasurement.gpio_state;

//     bool ButtonPressed = CHECK_BIT(gpio_state, 3); // Read button state

//     // Update button states based on handedness
//     if (this->handedness_ == Handedness::RIGHT)
//     {
//         GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, ButtonPressed, 0);
//         GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, ButtonPressed, 0);

//         // Optional system button handling
//         if (GetAsyncKeyState('O') & 0x8000)
//         {
//             GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, true, 0);
//             GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, true, 0);
//         }
//         else
//         {
//             GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, false, 0);
//             GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, false, 0);
//         }
//     }
//     else if (this->handedness_ == Handedness::LEFT)
//     {
//         GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_click_component_, ButtonPressed, 0);
//         GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_touch_component_, ButtonPressed, 0);
//     }

//     // Update the pose in SteamVR
//     pose.poseIsValid = true; // Mark the pose as valid for SteamVR
//     pose.result = vr::TrackingResult_Running_OK;
//     GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));

//     // Store the last pose for reference
//     this->last_pose_ = pose;
// }

void AmfitrackDriver::ControllerDevice::Update()
{
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

    auto pose = IVRDevice::MakeDefaultPose();

    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    if (!AMFITRACK.getDeviceActive(this->deviceID_))
    {
        pose.poseIsValid = false;
        return;
    }

    // lib_AmfiProt_Amfitrack_Pose_t tracker_pose;
    // AMFITRACK.getDevicePose(4, &tracker_pose);
    // vr::DriverPose_t source_pose = GetSourcePose(4, tracker_pose);

    lib_AmfiProt_Amfitrack_Pose_t position;
    AMFITRACK.getDevicePose(this->deviceID_, &position);

    // Apply fixed offsets like in the tracker to decouple from HMD movement
    vr::HmdVector3_t offset_position_controller = {
        position.position_x_in_m,   // X position (apply 10cm left shift)
        -position.position_y_in_m + 1.40f, // Y position (no change here)
        -position.position_z_in_m + 0.55f  // Z position (apply 60cm backward shift in one step)
    };

    // Set the position directly, no HMD involvement
    pose.vecPosition[0] = offset_position_controller.v[0];
    pose.vecPosition[1] = offset_position_controller.v[1];
    pose.vecPosition[2] = offset_position_controller.v[2];

    // Set rotation directly from the device's orientation, applying necessary quaternion adjustments
    pose.qRotation = {
        position.orientation_w,
        position.orientation_x,
        -position.orientation_y,
        -position.orientation_z};

    // Apply any additional fixed rotations (if required)
    vr::HmdQuaternion_t rotationQuatY = {0.7071068, 0, 0.7071068, 0}; // 90-degree Y rotation
    vr::HmdQuaternion_t rotationQuatX = {0.7071068, 0, 0, 0.7071068}; // 90-degree X rotation
    vr::HmdQuaternion_t rotationQuatZ = {0, 1, 0, 0};                 // 180-degree Z rotation

    // Combine the rotations
    pose.qRotation = MultiplyQuaternions(MultiplyQuaternions(pose.qRotation, rotationQuatY), rotationQuatX);
    pose.qRotation = MultiplyQuaternions(pose.qRotation, rotationQuatZ);

    pose.qRotation = HmdQuaternion_Normalize(pose.qRotation);

    // Handle button press states
    lib_AmfiProt_Amfitrack_Sensor_Measurement_t sensorMeasurement;
    AMFITRACK.getSensorMeasurements(this->deviceID_, &sensorMeasurement);
    uint16_t gpio_state = sensorMeasurement.gpio_state;

    bool ButtonPressed = CHECK_BIT(gpio_state, 3);

    // Update button states based on handedness
    if (this->handedness_ == Handedness::RIGHT)
    {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, ButtonPressed, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, ButtonPressed, 0);

        if (GetAsyncKeyState('O') & 0x8000)
        {
            GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, true, 0);
            GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, true, 0);
        }
        else
        {
            GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, false, 0);
            GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, false, 0);
        }
    }
    else if (this->handedness_ == Handedness::LEFT)
    {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_click_component_, ButtonPressed, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_touch_component_, ButtonPressed, 0);
    }

    // Update the pose in SteamVR
    GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
    this->last_pose_ = pose;
}

DeviceType AmfitrackDriver::ControllerDevice::GetDeviceType()
{
    return DeviceType::CONTROLLER;
}

AmfitrackDriver::ControllerDevice::Handedness AmfitrackDriver::ControllerDevice::GetHandedness()
{
    return this->handedness_;
}

vr::TrackedDeviceIndex_t AmfitrackDriver::ControllerDevice::GetDeviceIndex()
{
    return this->device_index_;
}

void AmfitrackDriver::ControllerDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void AmfitrackDriver::ControllerDevice::EnterStandby()
{
}

void *AmfitrackDriver::ControllerDevice::GetComponent(const char *pchComponentNameAndVersion)
{
    return nullptr;
}

void AmfitrackDriver::ControllerDevice::DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t AmfitrackDriver::ControllerDevice::GetPose()
{
    return last_pose_;
}
