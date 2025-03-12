#include "ControllerDevice.hpp"
#include <Windows.h>
#include <fstream>
#include "vrmath.h"
#include "TrackerDevice.hpp"
#include "PoseHelper.hpp"
#include "amfitrack_cpp_SDK/Amfitrack.hpp"


#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))
#define CHECK_ANALOG(var) float((var) >> 4)/powf(2.0,12)

enum ConfigMode_t
{
    SourceUpright = 0,
    SourceOnHMD = 5,
    SourceFreeMoving = 10,
};

static ConfigMode_t configValue = SourceUpright;
static bool isUnity = false;
static bool isUnrealEngine = false;

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
    // GetDriver()->GetProperties()->SetUint64Property(this->props_, vr::Prop_CurrentUniverseId_Uint64, 2);

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

    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/grip/touch", &this->grip_touch_component_);
    GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/grip/value", &this->grip_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
    GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/grip/force", &this->grip_force_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/system/click", &this->system_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/system/touch", &this->system_touch_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/trackpad/click", &this->trackpad_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/trackpad/touch", &this->trackpad_touch_component_);
    GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/trackpad/x", &this->trackpad_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/trackpad/y", &this->trackpad_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);

    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/thumbstick/click", &this->thumbstick_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/thumbstick/touch", &this->thumbstick_touch_component_);
    GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/thumbstick/x", &this->thumbstick_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    GetDriver()->GetInput()->CreateScalarComponent(this->props_, "/input/thumbstick/y", &this->thumbstick_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);

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

void AmfitrackDriver::ControllerDevice::RegisterButtonPress(uint16_t gpio_state)
{

    bool ButtonPressed_0 = CHECK_BIT(gpio_state, 0);
    bool ButtonPressed_1 = CHECK_BIT(gpio_state, 1);
    bool ButtonPressed_2 = CHECK_BIT(gpio_state, 2);
    bool ButtonPressed_3 = CHECK_BIT(gpio_state, 3);
    float analog_0 = CHECK_ANALOG(gpio_state);



    if (this->handedness_ == Handedness::RIGHT)
    {

        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, ButtonPressed_0, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, ButtonPressed_0, 0);

        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_click_component_, ButtonPressed_1, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_touch_component_, ButtonPressed_1, 0);

        GetDriver()->GetInput()->UpdateScalarComponent(this->trigger_value_component_, analog_0, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->trigger_touch_component_, false, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->trigger_click_component_, false, 0);

        GetDriver()->GetInput()->UpdateBooleanComponent(this->grip_touch_component_, ButtonPressed_2, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->grip_value_component_, 0.5f, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->grip_force_component_, 0.5f, 0);

        GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, ButtonPressed_3, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, ButtonPressed_3, 0);

        /*GetDriver()->GetInput()->UpdateScalarComponent(this->trackpad_x_component_, 0.5f, 0);
        GetDriver()->GetInput()->UpdateScalarComponent(this->trackpad_y_component_, 0.5f, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->trackpad_touch_component_, true, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->trackpad_click_component_, true, 0);*/

        GetDriver()->GetInput()->UpdateScalarComponent(this->thumbstick_x_component_, 0.5f, 0);
        GetDriver()->GetInput()->UpdateScalarComponent(this->thumbstick_y_component_, 0.5f, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->thumbstick_touch_component_, true, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->thumbstick_click_component_, true, 0);

        if (GetAsyncKeyState('O') & 0x8000)
        {
            GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, true, 0);
            GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, true, 0);
        }
        //else //COMPONENT NEEDS TO BE RESET IF ACTUAL BINARY BUTTON ISN'T INVOLVED!
        //{
        //    GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, false, 0);
        //    GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, false, 0);
        //}
    }
    else if (this->handedness_ == Handedness::LEFT)
    {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, ButtonPressed_0, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, ButtonPressed_0, 0);

        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_click_component_, ButtonPressed_1, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_touch_component_, ButtonPressed_1, 0);

        GetDriver()->GetInput()->UpdateScalarComponent(this->trigger_value_component_, analog_0, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->trigger_touch_component_, false, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->trigger_click_component_, false, 0);

        GetDriver()->GetInput()->UpdateBooleanComponent(this->grip_touch_component_, ButtonPressed_2, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->grip_value_component_, 0.5f, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->grip_force_component_, 0.5f, 0);

        GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, ButtonPressed_3, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, ButtonPressed_3, 0);

        /*GetDriver()->GetInput()->UpdateScalarComponent(this->trackpad_x_component_, 0.5f, 0);
        GetDriver()->GetInput()->UpdateScalarComponent(this->trackpad_y_component_, 0.5f, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->trackpad_touch_component_, true, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->trackpad_click_component_, true, 0);*/

        GetDriver()->GetInput()->UpdateScalarComponent(this->thumbstick_x_component_, 0.5f, 0);
        GetDriver()->GetInput()->UpdateScalarComponent(this->thumbstick_y_component_, 0.5f, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->thumbstick_touch_component_, true, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->thumbstick_click_component_, true, 0);
    }
}

vr::DriverPose_t AmfitrackDriver::ControllerDevice::ToDriverPose(AmfitrackDriver::VRPose &pose)
{
    vr::DriverPose_t out_pose = IVRDevice::MakeDefaultPose();

    out_pose.poseIsValid = true;

    out_pose.result = vr::ETrackingResult::TrackingResult_Running_OK;

    out_pose.qRotation.w = pose.Orientation.w;
    out_pose.qRotation.x = pose.Orientation.x;
    out_pose.qRotation.y = pose.Orientation.y;
    out_pose.qRotation.z = pose.Orientation.z;

    out_pose.vecPosition[0] = pose.Position.v[0];
    out_pose.vecPosition[1] = pose.Position.v[1];
    out_pose.vecPosition[2] = pose.Position.v[2];
    
    return out_pose;
}

void ConvertToSteamVRCoordinate(lib_AmfiProt_Amfitrack_Pose_t *input, lib_AmfiProt_Amfitrack_Pose_t *output)
{
    output->position_x_in_m = input->position_x_in_m;
    output->position_y_in_m = input->position_z_in_m;
    output->position_z_in_m = -input->position_y_in_m;

    output->orientation_x = input->orientation_x;
    output->orientation_y = input->orientation_z;
    output->orientation_z = -input->orientation_y;
    output->orientation_w = input->orientation_w;

    float Multiplier = 1;
    if (isUnity) Multiplier = 1000;
    else if (isUnrealEngine) Multiplier = 10;

    output->position_x_in_m *= Multiplier;
    output->position_y_in_m *= Multiplier;
    output->position_z_in_m *= Multiplier;
}


void AmfitrackDriver::ControllerDevice::Update()
{
    //TODO NOTICE COPIED INTO Tracker
    AmfitrackDriver::PoseHelper poseHelper;
    isUnity = vr::VRSettings()->GetBool("driver_amfitrack", "isUnity", nullptr);
    isUnrealEngine = vr::VRSettings()->GetBool("driver_amfitrack", "isUnrealEngine", nullptr);
    configValue = (ConfigMode_t)vr::VRSettings()->GetInt32("driver_amfitrack", "sourceMounting", nullptr);

    if (configValue == SourceUpright)
    {
        if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
            return;

        auto pose = IVRDevice::MakeDefaultPose();

        AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
        if (!AMFITRACK.getDeviceActive(this->deviceID_))
        {
            pose.poseIsValid = false;
            return;
        }

        lib_AmfiProt_Amfitrack_Pose_t position;
        lib_AmfiProt_Amfitrack_Pose_t updatedPosition;
        AMFITRACK.getDevicePose(this->deviceID_, &position);

        ConvertToSteamVRCoordinate(&position, &updatedPosition);

        pose.vecPosition[0] = updatedPosition.position_x_in_m;
        pose.vecPosition[1] = updatedPosition.position_y_in_m;
        pose.vecPosition[2] = updatedPosition.position_z_in_m;

        pose.qRotation.x = updatedPosition.orientation_x;
        pose.qRotation.y = updatedPosition.orientation_y;
        pose.qRotation.z = updatedPosition.orientation_z;
        pose.qRotation.w = updatedPosition.orientation_w;

        // Update in SteamVR
        GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
        this->last_pose_ = pose;
    }
    else if (configValue == SourceOnHMD)
    {
        if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
            return;

        if (this->device_index_ == 5)
        {
            auto pose = IVRDevice::MakeDefaultPose();
            pose.vecPosition[0] = 1.0f;
            pose.vecPosition[1] = 1.0f;
            pose.vecPosition[2] = 1.0f;

            pose.qRotation = { 1.0f, 0.0f, 0.0f, 0.0f};

            GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
            this->last_pose_ = pose;
            return;
        }

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
        vr::HmdVector3_t offset_position_controller = {position.position_x_in_m, -position.position_y_in_m, -position.position_z_in_m + 0.02f};
        vr::HmdVector3_t offset_position_source = {0, 0.1f, -0.1f};

        // Optimized position calculation using the new math functions
        vr::HmdVector3_t positionhmd_sensor = AddVectors(RotateVectorByQuaternion(offset_position_controller, hmd_orientation), hmd_position);
        vr::HmdVector3_t positionhmd_source = RotateVectorByQuaternion(offset_position_source, hmd_orientation);

        pose.vecPosition[0] = positionhmd_sensor.v[0] + positionhmd_source.v[0];
        pose.vecPosition[1] = positionhmd_sensor.v[1] + positionhmd_source.v[1];
        pose.vecPosition[2] = positionhmd_sensor.v[2] + positionhmd_source.v[2];

        // Optimized quaternion rotations
        pose.qRotation = {position.orientation_w, position.orientation_x, -position.orientation_y, -position.orientation_z};

        vr::HmdQuaternion_t rotationQuatY = {0.7071068, 0, 0.7071068, 0};
        vr::HmdQuaternion_t rotationQuatX = {0.7071068, 0, 0, 0.7071068};
        vr::HmdQuaternion_t rotationQuatZ = {0, 1, 0, 0};

        pose.qRotation = MultiplyQuaternions(MultiplyQuaternions(pose.qRotation, rotationQuatY), rotationQuatX);
        pose.qRotation = MultiplyQuaternions(pose.qRotation, rotationQuatZ);
        pose.qRotation = MultiplyQuaternions(hmd_orientation, pose.qRotation);

        lib_AmfiProt_Amfitrack_Sensor_Measurement_t sensorMeasurement;
        AMFITRACK.getSensorMeasurements(this->deviceID_, &sensorMeasurement);
        uint16_t gpio_state = sensorMeasurement.gpio_state;

        this->RegisterButtonPress(gpio_state);

        GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, pose, sizeof(vr::DriverPose_t));
        this->last_pose_ = pose;
    }
    else if (configValue == SourceFreeMoving)
    {
        AmfitrackDriver::VRPose pose;

        // --- Gather poses from SteamVR ---
        vr::TrackedDevicePose_t tracked_poses[vr::k_unMaxTrackedDeviceCount];
        vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.f, tracked_poses, vr::k_unMaxTrackedDeviceCount);

        // We assume the generic tracker is at index 16
        AmfitrackDriver::VRPose steamVR_tracker_pose{};

        vr::HmdVector3_t hmd_position = HmdVector3_From34Matrix(tracked_poses[16].mDeviceToAbsoluteTracking);
        steamVR_tracker_pose.Position.v[0] = hmd_position.v[0];
        steamVR_tracker_pose.Position.v[1] = hmd_position.v[1];
        steamVR_tracker_pose.Position.v[2] = hmd_position.v[2];

        // Acquire the Amfitrack singleton and fetch the generic tracker pose
        AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
        lib_AmfiProt_Amfitrack_Pose_t generic_tracker_pose{};
        AMFITRACK.getDevicePose(4, &generic_tracker_pose);

        AmfitrackDriver::VRPose generic_hmd_tracker{};
        generic_hmd_tracker.Position.v[0] = generic_tracker_pose.position_x_in_m;
        generic_hmd_tracker.Position.v[1] = generic_tracker_pose.position_y_in_m;
        generic_hmd_tracker.Position.v[2] = generic_tracker_pose.position_z_in_m;

        // --- Compute the source_position using GetSourcePose ---
        AmfitrackDriver::VRPose source_position = poseHelper.GetSourcePose(generic_hmd_tracker, steamVR_tracker_pose);

        // --- Fetch this controller’s Amfitrack pose for calculating final pose ---
        lib_AmfiProt_Amfitrack_Pose_t amfitrack_controller_pose{};
        AMFITRACK.getDevicePose(this->deviceID_, &amfitrack_controller_pose);

        AmfitrackDriver::VRPose controller_pose{};
        controller_pose.Orientation = {
            amfitrack_controller_pose.orientation_w,
            amfitrack_controller_pose.orientation_x,
            amfitrack_controller_pose.orientation_y,
            amfitrack_controller_pose.orientation_z};

        controller_pose.Position = {
            amfitrack_controller_pose.position_x_in_m,
            amfitrack_controller_pose.position_y_in_m,
            amfitrack_controller_pose.position_z_in_m};

        // --- Calculate the controller's pose in SteamVR space ---
        pose = poseHelper.CalculateControllerPose(source_position, controller_pose);

        vr::DriverPose_t out_pose = ToDriverPose(pose);

        // --- Button handling ---
        lib_AmfiProt_Amfitrack_Sensor_Measurement_t sensorMeasurement;
        AMFITRACK.getSensorMeasurements(this->deviceID_, &sensorMeasurement);
        uint16_t gpio_state = sensorMeasurement.gpio_state;

        this->RegisterButtonPress(gpio_state);

        // Update in SteamVR
        GetDriver()->GetDriverHost()->TrackedDevicePoseUpdated(this->device_index_, out_pose, sizeof(vr::DriverPose_t));
        this->last_pose_ = out_pose;
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
