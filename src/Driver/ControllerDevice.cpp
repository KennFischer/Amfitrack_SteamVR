#include "ControllerDevice.hpp"
#include <Windows.h>
#include "amfitrack_cpp_SDK/Amfitrack.hpp"

#define CHECK_BIT(var, pos) ((var) & (1 << (pos)))

AmfitrackDriver::ControllerDevice::ControllerDevice(uint8_t deviceId, std::string serial, ControllerDevice::Handedness handedness):
    deviceID_(deviceId),
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

void AmfitrackDriver::ControllerDevice::Update()
{
    if (this->device_index_ == vr::k_unTrackedDeviceIndexInvalid)
        return;

    // Check if this device was asked to be identified
    auto events = GetDriver()->GetOpenVREvents();
    for (auto event : events) {
        // Note here, event.trackedDeviceIndex does not necissarily equal this->device_index_, not sure why, but the component handle will match so we can just use that instead
        //if (event.trackedDeviceIndex == this->device_index_) {
        if (event.eventType == vr::EVREventType::VREvent_Input_HapticVibration) {
            if (event.data.hapticVibration.componentHandle == this->haptic_component_) {
                this->did_vibrate_ = true;
            }
        }
        //}
    }

    // Check if we need to keep vibrating
    if (this->did_vibrate_) {
        this->vibrate_anim_state_ += (GetDriver()->GetLastFrameTime().count()/1000.f);
        if (this->vibrate_anim_state_ > 1.0f) {
            this->did_vibrate_ = false;
            this->vibrate_anim_state_ = 0.0f;
        }
    }

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
		return;
	}

	lib_AmfiProt_Amfitrack_Pose_t position;
	AMFITRACK.getDevicePose(this->deviceID_, &position);

	pose.vecPosition[0] = -position.position_x_in_m;
	pose.vecPosition[1] = position.position_y_in_m;
	pose.vecPosition[2] = -position.position_z_in_m;

	pose.qRotation.w = position.orientation_w;
	pose.qRotation.x = -position.orientation_x;
	pose.qRotation.y = position.orientation_y;
	pose.qRotation.z = -position.orientation_z;

	std::stringstream x, y, z;
	x << static_cast<double>(pose.vecPosition[0]);
	y << static_cast<double>(pose.vecPosition[1]);
	z << static_cast<double>(pose.vecPosition[2]);
	std::string message = "Pose X: " + x.str() + " | Y: " + y.str() + " | Z: " + z.str();
	GetDriver()->Log(message.c_str());

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
    if (GetAsyncKeyState(0x45 /* E */) != 0) {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, true, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, true, 0);
    }
    else {
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_click_component_, false, 0);
        GetDriver()->GetInput()->UpdateBooleanComponent(this->a_button_touch_component_, false, 0);
    }

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

vr::EVRInitError AmfitrackDriver::ControllerDevice::Activate(uint32_t unObjectId)
{
    this->device_index_ = unObjectId;

    GetDriver()->Log("Activating controller " + this->serial_);

    // Get the properties handle
    auto props = GetDriver()->GetProperties()->TrackedDeviceToPropertyContainer(this->device_index_);

    // Setup inputs and outputs
    GetDriver()->GetInput()->CreateHapticComponent(props, "/output/haptic", &this->haptic_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/a/click", &this->a_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/a/touch", &this->a_button_touch_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/b/click", &this->b_button_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/b/touch", &this->b_button_touch_component_);

    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trigger/click", &this->trigger_click_component_);
    GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trigger/touch", &this->trigger_touch_component_);
    GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trigger/value", &this->trigger_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

    // GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/grip/touch", &this->grip_touch_component_);
    // GetDriver()->GetInput()->CreateScalarComponent(props, "/input/grip/value", &this->grip_value_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);
    // GetDriver()->GetInput()->CreateScalarComponent(props, "/input/grip/force", &this->grip_force_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedOneSided);

    // GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/click", &this->system_click_component_);
    // GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/system/touch", &this->system_touch_component_);

    // GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trackpad/click", &this->trackpad_click_component_);
    // GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/trackpad/touch", &this->trackpad_touch_component_); 
    // GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trackpad/x", &this->trackpad_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    // GetDriver()->GetInput()->CreateScalarComponent(props, "/input/trackpad/y", &this->trackpad_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    
    // GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/joystick/click", &this->joystick_click_component_);
    // GetDriver()->GetInput()->CreateBooleanComponent(props, "/input/joystick/touch", &this->joystick_touch_component_);
    // GetDriver()->GetInput()->CreateScalarComponent(props, "/input/joystick/x", &this->joystick_x_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);
    // GetDriver()->GetInput()->CreateScalarComponent(props, "/input/joystick/y", &this->joystick_y_component_, vr::EVRScalarType::VRScalarType_Absolute, vr::EVRScalarUnits::VRScalarUnits_NormalizedTwoSided);

    // Set some universe ID (Must be 2 or higher)
    GetDriver()->GetProperties()->SetUint64Property(props, vr::Prop_CurrentUniverseId_Uint64, 2);
    
    // Set up a model "number" (not needed but good to have)
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "amfitrack_controller");

    // Set up a render model path
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_RenderModelName_String, "vr_controller_vive_1_5");

    // Give SteamVR a hint at what hand this controller is for
    if (this->handedness_ == Handedness::LEFT) {
        GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_LeftHand);
    }
    else if (this->handedness_ == Handedness::RIGHT) {
        GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_RightHand);
    }
    else {
        GetDriver()->GetProperties()->SetInt32Property(props, vr::Prop_ControllerRoleHint_Int32, vr::ETrackedControllerRole::TrackedControllerRole_OptOut);
    }

    // Set controller profile
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_InputProfilePath_String, "{amfitrack}/input/amfitrack_controller_bindings.json");

    // Change the icon depending on which handedness this controller is using (ANY uses right)
    std::string controller_handedness_str = this->handedness_ == Handedness::LEFT ? "left" : "right";
    std::string controller_ready_file = "{amfitrack}/icons/controller_ready_" + controller_handedness_str + ".png";
    std::string controller_not_ready_file = "{amfitrack}/icons/controller_not_ready_" + controller_handedness_str + ".png";

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReady_String, controller_ready_file.c_str());

    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceOff_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearching_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceSearchingAlert_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceReadyAlert_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceNotReady_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceStandby_String, controller_not_ready_file.c_str());
    GetDriver()->GetProperties()->SetStringProperty(props, vr::Prop_NamedIconPathDeviceAlertLow_String, controller_not_ready_file.c_str());

    return vr::EVRInitError::VRInitError_None;
}

void AmfitrackDriver::ControllerDevice::Deactivate()
{
    this->device_index_ = vr::k_unTrackedDeviceIndexInvalid;
}

void AmfitrackDriver::ControllerDevice::EnterStandby()
{
}

void* AmfitrackDriver::ControllerDevice::GetComponent(const char* pchComponentNameAndVersion)
{
    return nullptr;
}

void AmfitrackDriver::ControllerDevice::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
{
    if (unResponseBufferSize >= 1)
        pchResponseBuffer[0] = 0;
}

vr::DriverPose_t AmfitrackDriver::ControllerDevice::GetPose()
{
    return last_pose_;
}
