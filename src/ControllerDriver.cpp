#include <ControllerDriver.h>
#include "amfitrack_cpp_SDK/Amfitrack.hpp"

#include <sstream>
#include <string>
#include <math.h>
#include <Windows.h>

ControllerDriver::ControllerDriver(uint8_t DeviceID, ControllerDriver::Handedness handedness) :
	deviceID_(DeviceID),
	handedness_(handedness)
{
}

EVRInitError ControllerDriver::Activate(uint32_t unObjectId)
{
	this->device_index_ = unObjectId; //unique ID for your driver

	PropertyContainerHandle_t props = VRProperties()->TrackedDeviceToPropertyContainer(this->device_index_); //this gets a container object where you store all the information about your driver

	VRDriverInput()->CreateBooleanComponent(props, "/input/a/click", &this->a_button_click_component_);
	VRDriverInput()->CreateBooleanComponent(props, "/input/a/touch", &this->a_button_touch_component_);

	VRDriverInput()->CreateBooleanComponent(props, "/input/b/click", &this->b_button_click_component_);
	VRDriverInput()->CreateBooleanComponent(props, "/input/b/touch", &this->b_button_touch_component_);

	VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/click", &this->trigger_click_component_);
	VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/touch", &this->trigger_touch_component_);

	VRProperties()->SetUint64Property(props, vr::Prop_CurrentUniverseId_Uint64, 2);

	// Set up a model "number" (not needed but good to have)
	VRProperties()->SetStringProperty(props, vr::Prop_ModelNumber_String, "amfitrack_controller");

	VRProperties()->SetStringProperty(props, Prop_RenderModelName_String, "vr_controller_vive_1_5");

	VRProperties()->SetBoolProperty(props, Prop_HasControllerComponent_Bool, true);

	// Give SteamVR a hint at what hand this controller is for
	if (this->handedness_ == Handedness::LEFT) {
		VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_LeftHand); //tells OpenVR what kind of device this is
	}
	else if (this->handedness_ == Handedness::RIGHT) {
		VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_RightHand); //tells OpenVR what kind of device this is
	}
	else {
		VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_OptOut); //tells OpenVR what kind of device this is
	}

	VRProperties()->SetStringProperty(props, Prop_InputProfilePath_String, "{Amfitrack}/input/controller_profile.json"); //tell OpenVR where to get your driver's Input Profile

	std::stringstream ss;
	ss << static_cast<int>(this->deviceID_);  // Convert to int for stream insertion
	std::string message = "Initialized device: " + ss.str();
	VRDriverLog()->Log(message.c_str());

	//VRDriverInput()->CreateScalarComponent(props, "/input/joystick/y", &joystickYHandle, EVRScalarType::VRScalarType_Absolute,
	//	EVRScalarUnits::VRScalarUnits_NormalizedTwoSided); //sets up handler you'll use to send joystick commands to OpenVR with, in the Y direction (forward/backward)
	//VRDriverInput()->CreateScalarComponent(props, "/input/trackpad/y", &trackpadYHandle, EVRScalarType::VRScalarType_Absolute,
	//	EVRScalarUnits::VRScalarUnits_NormalizedTwoSided); //sets up handler you'll use to send trackpad commands to OpenVR with, in the Y direction
	//VRDriverInput()->CreateScalarComponent(props, "/input/joystick/x", &joystickXHandle, EVRScalarType::VRScalarType_Absolute,
	//	EVRScalarUnits::VRScalarUnits_NormalizedTwoSided); //Why VRScalarType_Absolute? Take a look at the comments on EVRScalarType.
	//VRDriverInput()->CreateScalarComponent(props, "/input/trackpad/x", &trackpadXHandle, EVRScalarType::VRScalarType_Absolute,
	//	EVRScalarUnits::VRScalarUnits_NormalizedTwoSided); //Why VRScalarUnits_NormalizedTwoSided? Take a look at the comments on EVRScalarUnits.
	
	//The following properites are ones I tried out because I saw them in other samples, but I found they were not needed to get the sample working.
	//There are many samples, take a look at the openvr_header.h file. You can try them out.

	//VRProperties()->SetUint64Property(props, Prop_CurrentUniverseId_Uint64, 2);
	//VRProperties()->SetBoolProperty(props, Prop_HasControllerComponent_Bool, true);
	//VRProperties()->SetBoolProperty(props, Prop_NeverTracked_Bool, true);
	//VRProperties()->SetInt32Property(props, Prop_Axis0Type_Int32, k_eControllerAxis_TrackPad);
	//VRProperties()->SetInt32Property(props, Prop_Axis2Type_Int32, k_eControllerAxis_Joystick);
	//VRProperties()->SetStringProperty(props, Prop_SerialNumber_String, "Amfitrack_controler_serial");
	//VRProperties()->SetStringProperty(props, Prop_RenderModelName_String, "vr_controller_vive_1_5");
	//uint64_t availableButtons = ButtonMaskFromId(k_EButton_SteamVR_Touchpad) |
	//	ButtonMaskFromId(k_EButton_IndexController_JoyStick);
	//VRProperties()->SetUint64Property(props, Prop_SupportedButtons_Uint64, availableButtons);

	return VRInitError_None;
}

void normalizeQuaternion(HmdQuaternion_t& q)
{
	double norm = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	q.w /= norm;
	q.x /= norm;
	q.y /= norm;
	q.z /= norm;
}

HmdQuaternion_t rotate(HmdQuaternion_t a, HmdQuaternion_t b)
{
	HmdQuaternion_t result;
	result.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z; 
	result.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y; 
	result.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x; 
	result.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w; 

	return result;
}

DriverPose_t ControllerDriver::GetPose()
{
	AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
	DriverPose_t pose = { 0 }; //This Amfitrack doesn't use Pose, so this method is just returning a default Pose.
	if (!AMFITRACK.getDeviceActive(this->deviceID_))
	{
		std::stringstream ss;
		ss << static_cast<int>(this->deviceID_);  // Convert to int for stream insertion
		std::string message = "No Pose for this device: " + ss.str();
		VRDriverLog()->Log(message.c_str());
		pose.poseIsValid = false;
		return pose;
	}

	lib_AmfiProt_Amfitrack_Pose_t position;
	AMFITRACK.getDevicePose(this->deviceID_, &position);



	pose.deviceIsConnected = true;
	pose.poseIsValid = true;
	pose.result = TrackingResult_Running_OK;
	pose.willDriftInYaw = false;
	pose.shouldApplyHeadModel = false;
	pose.qDriverFromHeadRotation.w = pose.qWorldFromDriverRotation.w = pose.qRotation.w = 1.0;
	
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
	VRDriverLog()->Log(message.c_str());

	HmdQuaternion_t rotationQuat = {0};
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

	normalizeQuaternion(pose.qRotation);



	x << static_cast<double>(pose.qRotation.x);
	y << static_cast<double>(pose.qRotation.y);
	z << static_cast<double>(pose.qRotation.z);
	message = "Quat X: " + x.str() + " | Y: " + y.str() + " | Z: " + z.str();
	VRDriverLog()->Log(message.c_str());

	return pose;
}

void ControllerDriver::RunFrame()
{
	VRServerDriverHost()->TrackedDevicePoseUpdated(this->device_index_, GetPose(), sizeof(vr::DriverPose_t));

	if (GetAsyncKeyState(0x41 /* E */) != 0) {
		VRDriverInput()->UpdateBooleanComponent(this->a_button_click_component_, true, 0);
		VRDriverInput()->UpdateBooleanComponent(this->a_button_touch_component_, true, 0);
	}
	else {
		VRDriverInput()->UpdateBooleanComponent(this->a_button_click_component_, false, 0);
		VRDriverInput()->UpdateBooleanComponent(this->a_button_touch_component_, false, 0);
	}

	if (GetAsyncKeyState(0x51 /* Q */) != 0) {
		VRDriverInput()->UpdateBooleanComponent(this->b_button_click_component_, true, 0);
		VRDriverInput()->UpdateBooleanComponent(this->b_button_touch_component_, true, 0);
	}
	else {
		VRDriverInput()->UpdateBooleanComponent(this->b_button_click_component_, false, 0);
		VRDriverInput()->UpdateBooleanComponent(this->b_button_touch_component_, false, 0);
	}

	if (GetAsyncKeyState(0x44 /* D */) != 0) {
		VRDriverInput()->UpdateBooleanComponent(this->trigger_click_component_, true, 0);
		VRDriverInput()->UpdateBooleanComponent(this->trigger_touch_component_, true, 0);
	}
	else {
		VRDriverInput()->UpdateBooleanComponent(this->trigger_click_component_, false, 0);
		VRDriverInput()->UpdateBooleanComponent(this->trigger_touch_component_, false, 0);
	}
}

void ControllerDriver::Deactivate()
{
	this->device_index_ = k_unTrackedDeviceIndexInvalid;
}

void* ControllerDriver::GetComponent(const char* pchComponentNameAndVersion)
{
	//I found that if this method just returns null always, it works fine. But I'm leaving the if statement in since it doesn't hurt.
	//Check out the IVRDriverInput_Version declaration in openvr_driver.h. You can search that file for other _Version declarations 
	//to see other components that are available. You could also put a log in this class and output the value passed into this 
	//method to see what OpenVR is looking for.
	if (strcmp(IVRDriverInput_Version, pchComponentNameAndVersion) == 0)
	{
		return this;
	}
	return NULL;
}

void ControllerDriver::EnterStandby() {}

void ControllerDriver::DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) 
{
	if (unResponseBufferSize >= 1)
	{
		pchResponseBuffer[0] = 0;
	}
}
