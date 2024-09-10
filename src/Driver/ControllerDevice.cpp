#include "ControllerDevice.hpp"
#include <Windows.h>
#include "amfitrack_cpp_SDK/Amfitrack.hpp"
#include "vrmath.h"

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

vr::HmdQuaternion_t rotate(vr::HmdQuaternion_t a, vr::HmdQuaternion_t b)
{
    vr::HmdQuaternion_t result;
    result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
    result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
    result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
    result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;

    return result;
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
    GetDriver()->GetInput()->CreateHapticComponent(this->props_, "/output/haptic", &this->haptic_component_);

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

    // GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/system/click", &this->system_click_component_);
    // GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/system/touch", &this->system_touch_component_);

    // GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/trackpad/click", &this->trackpad_click_component_);
    // GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/trackpad/touch", &this->trackpad_touch_component_);
    // GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/trackpad/x", &this->trackpad_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    // GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/trackpad/y", &this->trackpad_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);

    // GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/joystick/click", &this->joystick_click_component_);
    // GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/joystick/touch", &this->joystick_touch_component_);
    // GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/joystick/x", &this->joystick_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    // GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/joystick/y", &this->joystick_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);

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

void AmfitrackDriver::ControllerDevice::Update()
{
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

    // Check if this device was asked to be identified
    auto events = GetDriver()->GetOpenVREvents();
    for (auto event : events)
    {
        // Note here, event.trackedDeviceIndex does not necissarily equal this->device_index_, not sure why, but the component handle will match so we can just use that instead
        // if (event.trackedDeviceIndex == this->device_index_) {
        if (event.eventType == vr::EVREventType::VREvent_Input_HapticVibration)
        {
            if (event.data.hapticVibration.componentHandle == this->haptic_component_)
            {
                this->did_vibrate_ = true;
            }
        }
        //}
    }

    // Check if we need to keep vibrating
    if (this->did_vibrate_)
    {
        this->vibrate_anim_state_ += (GetDriver()->GetLastFrameTime().count() / 1000.f);
        if (this->vibrate_anim_state_ > 1.0f)
        {
            this->did_vibrate_ = false;
            this->vibrate_anim_state_ = 0.0f;
        }
    }
    vr::TrackedDevicePose_t hmd_pose{};

    // GetRawTrackedDevicePoses expects an array.
    // We only want the hmd pose, which is at index 0 of the array so we can just pass the struct in directly, instead of in an array
    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.f, &hmd_pose, 1);

    // Get the position of the hmd from the 3x4 matrix GetRawTrackedDevicePoses returns
    vr::HmdVector3_t hmd_position = HmdVector3_From34Matrix(hmd_pose.mDeviceToAbsoluteTracking);
    // Get the orientation of the hmd from the 3x4 matrix GetRawTrackedDevicePoses returns
    vr::HmdQuaternion_t hmd_orientation = HmdQuaternion_FromMatrix(hmd_pose.mDeviceToAbsoluteTracking);

    std::stringstream hmd_x, hmd_y, hmd_z;
    hmd_x << static_cast<double>(hmd_position.v[0]);
    hmd_y << static_cast<double>(hmd_position.v[1]);
    hmd_z << static_cast<double>(hmd_position.v[2]);
    std::string hmd_message = "HMD Pose hmd_X: " + hmd_x.str() + " | hmd_Y: " + hmd_y.str() + " | hmd_Z: " + hmd_z.str();
    GetDriver()->Log(hmd_message);

    // Setup pose for this frame
    auto pose = IVRDevice::MakeDefaultPose();

    // Find a HMD
    auto devices = GetDriver()->GetDevices();
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    if (!AMFITRACK.getDeviceActive(this->deviceID_))
    {
        std::stringstream ss;
        ss << static_cast<int>(this->deviceID_); // Convert to int for stream insertion
        std::string message = "No Pose for this device: " + ss.str();
        GetDriver()->Log(message.c_str());
        pose.poseIsValid = false;
        // this->Deactivate();
        return;
    }

    lib_AmfiProt_Amfitrack_Pose_t position;
    AMFITRACK.getDevicePose(this->deviceID_, &position);

    vr::HmdVector3_t offset_position =
    {
        position.position_x_in_m,
        -position.position_y_in_m,
        -position.position_z_in_m,
    };

    // Rotate our offset by the hmd quaternion (so the controllers are always facing towards us), and add then add the position of the hmd to put it into position.
    const vr::HmdVector3_t positionhmd_sensor = hmd_position + (offset_position * hmd_orientation);

    // copy our position to our pose
    pose.vecPosition[0] = positionhmd_sensor.v[0];
    pose.vecPosition[1] = positionhmd_sensor.v[1];
    pose.vecPosition[2] = positionhmd_sensor.v[2];

    // pitch the controller 90 degrees so the face of the controller is facing towards us
    pose.qRotation.w = position.orientation_w;
    pose.qRotation.x = position.orientation_x;
    pose.qRotation.y = -position.orientation_y;
    pose.qRotation.z = -position.orientation_z;

    std::stringstream x, y, z;
    x << static_cast<double>(pose.vecPosition[0]);
    y << static_cast<double>(pose.vecPosition[1]);
    z << static_cast<double>(pose.vecPosition[2]);
    std::string message = "Pose X: " + x.str() + " | Y: " + y.str() + " | Z: " + z.str();
    GetDriver()->Log(std::to_string(this->deviceID_) + message);

    vr::HmdQuaternion_t rotationQuat = {0};
    // Rotate 90 degrees around Y-axis
    rotationQuat.y = 0.7071068;
    rotationQuat.w = 0.7071068;
    pose.qRotation = rotate(pose.qRotation, rotationQuat);

    // Rotate 90 degrees around X-axis
    rotationQuat.x = 0;
    rotationQuat.y = 0;
    rotationQuat.z = -0.7071068;
    rotationQuat.w = 0.7071068;
    pose.qRotation = rotate(pose.qRotation, rotationQuat);

    // Rotate 180 degrees around the Z-axis
    rotationQuat.x = 0;
    rotationQuat.y = 1;
    rotationQuat.z = 0;
    rotationQuat.w = 0;
    pose.qRotation = rotate(pose.qRotation, rotationQuat);

    pose.qRotation = hmd_orientation * pose.qRotation;

    lib_AmfiProt_Amfitrack_Sensor_Measurement_t sensorMeasurement;
    AMFITRACK.getSensorMeasurements(this->deviceID_, &sensorMeasurement);
    uint16_t gpio_state = sensorMeasurement.gpio_state;

    bool ButtonPressed = CHECK_BIT(gpio_state, 3);

    if (this->handedness_ == Handedness::RIGHT)
    {
        if (ButtonPressed)
        {
            GetDriver()->Log("Right sensor button pressed");
        }
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, ButtonPressed, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, ButtonPressed, 0);
    }
    else if (this->handedness_ == Handedness::LEFT)
    {
        if (ButtonPressed)
        {
            GetDriver()->Log("Left sensor button pressed");
        }
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_click_component_, ButtonPressed, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_touch_component_, ButtonPressed, 0);
    }

    // Check if we need to press any buttons (I am only hooking up the A button here but the process is the same for the others)
    // You will still need to go into the games button bindings and hook up each one (ie. a to left click, b to right click, etc.) for them to work properly
    // if (GetAsyncKeyState(0x45 /* E */) != 0) {
    //     GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, true, 0);
    //     GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, true, 0);
    // }
    // else {
    //     GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, false, 0);
    //     GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, false, 0);
    // }

    // Post pose
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
