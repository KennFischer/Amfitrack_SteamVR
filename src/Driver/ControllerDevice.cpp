#include "ControllerDevice.hpp"
#include <Windows.h>
#include <fstream>
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
//     if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
//         return;

//     auto pose = IVRDevice::MakeDefaultPose();

//     AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
//     if (!AMFITRACK.getDeviceActive(this->deviceID_))
//     {
//         pose.poseIsValid = false;
//         return;
//     }

//     lib_AmfiProt_Amfitrack_Pose_t tracker_pose;
//     AMFITRACK.getDevicePose(4, &tracker_pose);

//     lib_AmfiProt_Amfitrack_Pose_t position;
//     AMFITRACK.getDevicePose(this->deviceID_, &position);

//     // Apply fixed offsets like in the tracker to decouple from HMD movement
//     vr::HmdVector3_t offset_position_controller = {
//         position.position_x_in_m,          // X position (apply 10cm left shift)
//         -position.position_y_in_m + 1.40f, // Y position (no change here)
//         -position.position_z_in_m + 0.55f  // Z position (apply 60cm backward shift in one step)
//     };

//     // Set the position directly, no HMD involvement
//     pose.vecPosition[0] = offset_position_controller.v[0];
//     pose.vecPosition[1] = offset_position_controller.v[1];
//     pose.vecPosition[2] = offset_position_controller.v[2];

//     // Set rotation directly from the device's orientation, applying necessary quaternion adjustments
//     pose.qRotation = {
//         position.orientation_w,
//         position.orientation_x,
//         -position.orientation_y,
//         -position.orientation_z};

//     // Apply any additional fixed rotations (if required)
//     vr::HmdQuaternion_t rotationQuatY = {0.7071068, 0, 0.7071068, 0}; // 90-degree Y rotation
//     vr::HmdQuaternion_t rotationQuatX = {0.7071068, 0, 0, 0.7071068}; // 90-degree X rotation
//     vr::HmdQuaternion_t rotationQuatZ = {0, 1, 0, 0};                 // 180-degree Z rotation

//     // Combine the rotations
//     pose.qRotation = MultiplyQuaternions(MultiplyQuaternions(pose.qRotation, rotationQuatY), rotationQuatX);
//     pose.qRotation = MultiplyQuaternions(pose.qRotation, rotationQuatZ);

//     pose.qRotation = HmdQuaternion_Normalize(pose.qRotation);

//     // Handle button press states
//     lib_AmfiProt_Amfitrack_Sensor_Measurement_t sensorMeasurement;
//     AMFITRACK.getSensorMeasurements(this->deviceID_, &sensorMeasurement);
//     uint16_t gpio_state = sensorMeasurement.gpio_state;

//     bool ButtonPressed = CHECK_BIT(gpio_state, 3);

//     // Update button states based on handedness
//     if (this->handedness_ == Handedness::RIGHT)
//     {
//         GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, ButtonPressed, 0);
//         GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, ButtonPressed, 0);

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
//     GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
//     this->last_pose_ = pose;
// }

void AmfitrackDriver::ControllerDevice::Update()
{

    int configValue = 1;
    if(configValue==0){
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

    vr::TrackedDevicePose_t hmd_pose{};
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.f, &hmd_pose, 1);

    vr::HmdVector3_t hmd_position = HmdVector3_From34Matrix(hmd_pose.mDeviceToAbsoluteTracking);
    vr::HmdQuaternion_t hmd_orientation = HmdQuaternion_FromMatrix(hmd_pose.mDeviceToAbsoluteTracking);

    auto pose = IVRDevice::MakeDefaultPose();

    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    if (!AMFITRACK.getDeviceActive(this->deviceID_))
    {
        pose.poseIsValid = false;
        return;
    }

    lib_AmfiProt_Amfitrack_Pose_t position;
    AMFITRACK.getDevicePose(this->deviceID_, &position);

    // Offset and position
    vr::HmdVector3_t offset_position_controller = { position.position_x_in_m, -position.position_y_in_m, -position.position_z_in_m + 0.02f };
    vr::HmdVector3_t offset_position_source = { 0, 0.1f, -0.1f };

    // Optimized position calculation using the new math functions
    vr::HmdVector3_t positionhmd_sensor = AddVectors(RotateVectorByQuaternion(offset_position_controller, hmd_orientation), hmd_position);
    vr::HmdVector3_t positionhmd_source = RotateVectorByQuaternion(offset_position_source, hmd_orientation);

    pose.vecPosition[0] = positionhmd_sensor.v[0] + positionhmd_source.v[0];
    pose.vecPosition[1] = positionhmd_sensor.v[1] + positionhmd_source.v[1];
    pose.vecPosition[2] = positionhmd_sensor.v[2] + positionhmd_source.v[2];

    // Optimized quaternion rotations
    pose.qRotation = { position.orientation_w, position.orientation_x, -position.orientation_y, -position.orientation_z };

    vr::HmdQuaternion_t rotationQuatY = { 0.7071068, 0, 0.7071068, 0 };
    vr::HmdQuaternion_t rotationQuatX = { 0.7071068, 0, 0, 0.7071068 };
    vr::HmdQuaternion_t rotationQuatZ = { 0, 1, 0, 0 };

    pose.qRotation = MultiplyQuaternions(MultiplyQuaternions(pose.qRotation, rotationQuatY), rotationQuatX);
    pose.qRotation = MultiplyQuaternions(pose.qRotation, rotationQuatZ);
    pose.qRotation = MultiplyQuaternions(hmd_orientation, pose.qRotation);

    lib_AmfiProt_Amfitrack_Sensor_Measurement_t sensorMeasurement;
    AMFITRACK.getSensorMeasurements(this->deviceID_, &sensorMeasurement);
    uint16_t gpio_state = sensorMeasurement.gpio_state;

    bool ButtonPressed = CHECK_BIT(gpio_state, 3);

    if (this->handedness_ == Handedness::RIGHT)
    {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, ButtonPressed, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, ButtonPressed, 0);
    }
    else if (this->handedness_ == Handedness::LEFT)
    {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_click_component_, ButtonPressed, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_touch_component_, ButtonPressed, 0);
    }

    GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
    this->last_pose_ = pose;
    }
    // 
    else if(configValue==1){
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
