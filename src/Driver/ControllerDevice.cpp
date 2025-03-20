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
    SourceOnHMD = 1,
    SourceFreeMoving = 2,
};

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

    GetDriver()->GetInput()->CreateBooleanComponent(this->props_, "/input/grip/touch", &this->grip_click_component_);
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
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, ButtonPressed_1, 0);

        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_click_component_, ButtonPressed_1, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_touch_component_, ButtonPressed_0, 0);

        GetDriver()->GetInput()->UpdateScalarComponent(this->trigger_value_component_, analog_0, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->trigger_touch_component_, false, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->trigger_click_component_, false, 0);

        //GetDriver()->GetInput()->UpdateBooleanComponent(this->grip_click_component_, ButtonPressed_2, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->grip_touch_component_, ButtonPressed_2, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->grip_value_component_, 0.5f, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->grip_force_component_, 0.5f, 0);

        //GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, false, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, ButtonPressed_3, 0);

        //GetDriver()->GetInput()->UpdateScalarComponent(this->trackpad_x_component_, 0.5f, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->trackpad_y_component_, 0.5f, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->trackpad_click_component_, ButtonPressed_2, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->trackpad_touch_component_, ButtonPressed_2, 0);

        //GetDriver()->GetInput()->UpdateScalarComponent(this->thumbstick_x_component_, 0.5f, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->thumbstick_y_component_, 0.5f, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->thumbstick_touch_component_, ButtonPressed_3, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->thumbstick_click_component_, ButtonPressed_3, 0);

        if (GetAsyncKeyState('O') & 0x8000)
        {
            GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, true, 0);
            GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, true, 0);
        }
        else //COMPONENT NEEDS TO BE RESET IF ACTUAL BINARY BUTTON ISN'T INVOLVED!
        {
            GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, false, 0);
            GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, false, 0);
        }
    }
    else if (this->handedness_ == Handedness::LEFT)
    {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, ButtonPressed_1, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, ButtonPressed_1, 0);

        GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_click_component_, ButtonPressed_0, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->b_button_touch_component_, ButtonPressed_0, 0);

        GetDriver()->GetInput()->UpdateScalarComponent(this->trigger_value_component_, analog_0, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->trigger_touch_component_, false, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->trigger_click_component_, false, 0);

        //GetDriver()->GetInput()->UpdateBooleanComponent(this->grip_click_component_, ButtonPressed_2, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->grip_touch_component_, ButtonPressed_2, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->grip_value_component_, 0.5f, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->grip_force_component_, 0.5f, 0);

        //GetDriver()->GetInput()->UpdateBooleanComponent(this->system_click_component_, false, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->system_touch_component_, ButtonPressed_3, 0);

        //GetDriver()->GetInput()->UpdateScalarComponent(this->trackpad_x_component_, 0.5f, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->trackpad_y_component_, 0.5f, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->trackpad_click_component_, ButtonPressed_2, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->trackpad_touch_component_, ButtonPressed_2, 0);

        //GetDriver()->GetInput()->UpdateScalarComponent(this->thumbstick_x_component_, 0.5f, 0);
        //GetDriver()->GetInput()->UpdateScalarComponent(this->thumbstick_y_component_, 0.5f, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->thumbstick_touch_component_, ButtonPressed_3, 0);
        //GetDriver()->GetInput()->UpdateBooleanComponent(this->thumbstick_click_component_, ButtonPressed_3, 0);
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
    //Goes from right handed z-up, to 
    output->position_x_in_m = -input->position_y_in_m;
    output->position_y_in_m = input->position_z_in_m;
    output->position_z_in_m = -input->position_x_in_m;

    output->orientation_x = -input->orientation_y;
    output->orientation_y = input->orientation_z;
    output->orientation_z = -input->orientation_x;
    output->orientation_w = input->orientation_w;
}


void AmfitrackDriver::ControllerDevice::Update()
{
    //TODO NOTICE MUST BE COPIED INTO Tracker FROM CONTROLLER.  YES, THEY SHOULD JUST both INHERIET FROM ONE CLASS
    AmfitrackDriver::PoseHelper poseHelper;

    //isUnity = vr::VRSettings()->GetBool("driver_amfitrack", "isUnity", nullptr);
    //isUnrealEngine = vr::VRSettings()->GetBool("driver_amfitrack", "isUnrealEngine", nullptr);
    //configValue = (ConfigMode_t)vr::VRSettings()->GetInt32("driver_amfitrack", "sourceMounting", nullptr);
    ConfigMode_t ConfigMode = (ConfigMode_t)vr::VRSettings()->GetInt32("driver_amfitrack", "source_mode", nullptr);

    float attachment_position_HMD_x = 0.0f;
    float attachment_position_HMD_y = 0.0f;
    float attachment_position_HMD_z = 0.0f;
    float attachment_orientation_HMD_x = 0.0f;
    float attachment_orientation_HMD_y = 0.0f;
    float attachment_orientation_HMD_z = 0.0f;

    if (ConfigMode == SourceUpright)
    {
        attachment_position_HMD_x = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_position_HMD_x_Static_Source", nullptr);
        attachment_position_HMD_y = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_position_HMD_y_Static_Source", nullptr);
        attachment_position_HMD_z = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_position_HMD_z_Static_Source", nullptr);
        attachment_orientation_HMD_x = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_orientation_HMD_x_Static_Source", nullptr);
        attachment_orientation_HMD_y = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_orientation_HMD_y_Static_Source", nullptr);
        attachment_orientation_HMD_z = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_orientation_HMD_z_Static_Source", nullptr);
    }
    else if (ConfigMode == SourceOnHMD)
    {
        attachment_position_HMD_x = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_position_HMD_x_HMD_Source", nullptr);
        attachment_position_HMD_y = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_position_HMD_y_HMD_Source", nullptr);
        attachment_position_HMD_z = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_position_HMD_z_HMD_Source", nullptr);
        attachment_orientation_HMD_x = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_orientation_HMD_x_HMD_Source", nullptr);
        attachment_orientation_HMD_y = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_orientation_HMD_y_HMD_Source", nullptr);
        attachment_orientation_HMD_z = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_orientation_HMD_z_HMD_Source", nullptr);
    }
    else if (ConfigMode == SourceFreeMoving)
    {
        attachment_position_HMD_x = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_position_HMD_x_Source_Free_Moving", nullptr);
        attachment_position_HMD_y = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_position_HMD_y_Source_Free_Moving", nullptr);
        attachment_position_HMD_z = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_position_HMD_z_Source_Free_Moving", nullptr);
        attachment_orientation_HMD_x = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_orientation_HMD_x_Source_Free_Moving", nullptr);
        attachment_orientation_HMD_y = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_orientation_HMD_y_Source_Free_Moving", nullptr);
        attachment_orientation_HMD_z = vr::VRSettings()->GetFloat("driver_amfitrack", "attachment_orientation_HMD_z_Source_Free_Moving", nullptr);
    }

    int attachment_ID = 9;

    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

    //lib_AmfiProt_Amfitrack_Pose_t sensor_pose_steam;

    vr::HmdVector3_t sensor_position_steam;
    vr::HmdQuaternion_t sensor_orientation_steam;

    auto pose = IVRDevice::MakeDefaultPose();

    AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
    if (!AMFITRACK.getDeviceActive(this->deviceID_))
    {
        pose.poseIsValid = false;
        return;
    }

    //Get buttons Presses
    lib_AmfiProt_Amfitrack_Sensor_Measurement_t sensorMeasurement;
    AMFITRACK.getSensorMeasurements(this->deviceID_, &sensorMeasurement);
    uint16_t gpio_state = sensorMeasurement.gpio_state;
    this->RegisterButtonPress(gpio_state);

    lib_AmfiProt_Amfitrack_Pose_t raw_sensor_pose_source;
    lib_AmfiProt_Amfitrack_Pose_t sensor_pose_source;
    AMFITRACK.getDevicePose(this->deviceID_, &raw_sensor_pose_source);
    ConvertToSteamVRCoordinate(&raw_sensor_pose_source, &sensor_pose_source);
    vr::HmdVector3_t attachment_position_steam = { 0, 0, 0 };
    vr::HmdQuaternion_t attachment_orientation_steam = { 1.0f, 0, 0, 0 };
    vr::HmdVector3_t source_position_steam = { 0, 0, 0 };
    vr::HmdQuaternion_t source_orientation_steam = { 1.0f, 0, 0, 0 };
    vr::HmdVector3_t sensor_position_source = { sensor_pose_source.position_x_in_m, sensor_pose_source.position_y_in_m, sensor_pose_source.position_z_in_m };
    vr::HmdQuaternion_t sensor_orientation_source = { sensor_pose_source.orientation_w, sensor_pose_source.orientation_x, sensor_pose_source.orientation_y, sensor_pose_source.orientation_z };

    //Determine Attachment Pose in SteamVR
    if (ConfigMode == SourceUpright)
    {
        //In this case the attachment point isn't relative to the HMD, so the name is a bit of misnomer.
        attachment_position_steam = { attachment_position_HMD_x, attachment_position_HMD_y, attachment_position_HMD_z };
        attachment_orientation_steam = EulerToQuaternion(attachment_orientation_HMD_x, attachment_orientation_HMD_y, attachment_orientation_HMD_z);
    }
    else if (ConfigMode == SourceOnHMD || ConfigMode == SourceFreeMoving)
    {
        vr::TrackedDevicePose_t hmd_pose{};
        vr::VRServerDriverHost()->GetRawTrackedDevicePoses(0.f, &hmd_pose, 1);

        vr::HmdVector3_t hmd_position_steam = HmdVector3_From34Matrix(hmd_pose.mDeviceToAbsoluteTracking);
        vr::HmdQuaternion_t hmd_orientation_steam = HmdQuaternion_FromMatrix(hmd_pose.mDeviceToAbsoluteTracking);

        vr::HmdVector3_t attachment_position_HMD = { attachment_position_HMD_x, attachment_position_HMD_y, attachment_position_HMD_z };

        vr::HmdQuaternion_t attachment_orientation_HMD = EulerToQuaternion(attachment_orientation_HMD_x, attachment_orientation_HMD_y, attachment_orientation_HMD_z);
        attachment_position_steam = AddVectors(RotateVectorByQuaternion(attachment_position_HMD, hmd_orientation_steam), hmd_position_steam);
        attachment_orientation_steam = MultiplyQuaternions(hmd_orientation_steam, attachment_orientation_HMD);
    }

    //Determine Source Pose in SteamVR
    if (ConfigMode == SourceUpright || ConfigMode == SourceOnHMD)
    {
        source_position_steam = attachment_position_steam;
        source_orientation_steam = attachment_orientation_steam;
    }
    else if (ConfigMode == SourceFreeMoving)
    {
        lib_AmfiProt_Amfitrack_Pose_t raw_attachment_pose_source;
        lib_AmfiProt_Amfitrack_Pose_t attachment_pose_source;
        AMFITRACK.getDevicePose(attachment_ID, &raw_attachment_pose_source);
        ConvertToSteamVRCoordinate(&raw_attachment_pose_source, &attachment_pose_source);
        vr::HmdVector3_t attachment_position_source_inv = { -attachment_pose_source.position_x_in_m, -attachment_pose_source.position_y_in_m, -attachment_pose_source.position_z_in_m };
        vr::HmdQuaternion_t attachment_orientation_source = { attachment_pose_source.orientation_w, attachment_pose_source.orientation_x, attachment_pose_source.orientation_y, attachment_pose_source.orientation_z };
        //Invert
        vr::HmdQuaternion_t source_orientation_attachment = { attachment_orientation_source.w, -attachment_orientation_source.x, -attachment_orientation_source.y, -attachment_orientation_source.z };
        vr::HmdVector3_t source_position_attachment = RotateVectorByQuaternion(attachment_position_source_inv, source_orientation_attachment);



        source_position_steam = AddVectors(RotateVectorByQuaternion(source_position_attachment, attachment_orientation_steam), attachment_position_steam);
        source_orientation_steam = MultiplyQuaternions(attachment_orientation_steam, source_orientation_attachment);
    }

    //Determine Sensor position in SteamVR
    sensor_position_steam = AddVectors(RotateVectorByQuaternion(sensor_position_source, source_orientation_steam), source_position_steam);
    sensor_orientation_steam = MultiplyQuaternions(source_orientation_steam, sensor_orientation_source);

    if ((this->deviceID_ == attachment_ID) && (ConfigMode == SourceFreeMoving))
    {
        //Present source pose instead og
        sensor_position_steam.v[0] = source_position_steam.v[0];
        sensor_position_steam.v[1] = source_position_steam.v[1];
        sensor_position_steam.v[2] = source_position_steam.v[2];

        sensor_orientation_steam.w = source_orientation_steam.w;
        sensor_orientation_steam.x = source_orientation_steam.x;
        sensor_orientation_steam.y = source_orientation_steam.y;
        sensor_orientation_steam.z = source_orientation_steam.z;
    }
    if (vr::VRSettings()->GetBool("driver_amfitrack", "display_Attachment_pose", nullptr))
    {
        sensor_position_steam.v[0] = attachment_position_steam.v[0];
        sensor_position_steam.v[1] = attachment_position_steam.v[1];
        sensor_position_steam.v[2] = attachment_position_steam.v[2];

        sensor_orientation_steam.w = attachment_orientation_steam.w;
        sensor_orientation_steam.x = attachment_orientation_steam.x;
        sensor_orientation_steam.y = attachment_orientation_steam.y;
        sensor_orientation_steam.z = attachment_orientation_steam.z;
    }

    pose.vecPosition[0] = sensor_position_steam.v[0];
    pose.vecPosition[1] = sensor_position_steam.v[1];
    pose.vecPosition[2] = sensor_position_steam.v[2];

    pose.qRotation.x = sensor_orientation_steam.x;
    pose.qRotation.y = sensor_orientation_steam.y;
    pose.qRotation.z = sensor_orientation_steam.z;
    pose.qRotation.w = sensor_orientation_steam.w;

    // Update in SteamVR
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
