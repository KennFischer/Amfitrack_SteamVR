#include <ControllerDriver.h>
#include "amfitrack_cpp_SDK/Amfitrack.hpp"

#include <sstream>
#include <string>

ControllerDriver::ControllerDriver(uint8_t DeviceID, ControllerDriver::Handedness handedness) :
	deviceID_(DeviceID),
	handedness_(handedness)
{
}

EVRInitError ControllerDriver::Activate(uint32_t unObjectId)
{
	this->device_index_ = unObjectId; //unique ID for your driver

	PropertyContainerHandle_t props = VRProperties()->TrackedDeviceToPropertyContainer(this->device_index_); //this gets a container object where you store all the information about your driver

	VRProperties()->SetStringProperty(props, Prop_InputProfilePath_String, "{Amfitrack}/input/controller_profile.json"); //tell OpenVR where to get your driver's Input Profile
	VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, ETrackedControllerRole::TrackedControllerRole_LeftHand); //tells OpenVR what kind of device this is

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

DriverPose_t ControllerDriver::GetPose()
{
	AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
	DriverPose_t pose = { 0 }; //This Amfitrack doesn't use Pose, so this method is just returning a default Pose.
	if (AMFITRACK.getDeviceActive(this->deviceID_))
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

	std::stringstream x, y, z;
	x << static_cast<int>(position.position_x_in_m);
	y << static_cast<int>(position.position_y_in_m);
	z << static_cast<int>(position.position_z_in_m);
	std::string message = "X: " + x.str() + " | Y: " + y.str() + " | Z: " + z.str();
	VRDriverLog()->Log(message.c_str());

	pose.vecPosition[0] = position.position_x_in_m;
	pose.vecPosition[1] = position.position_z_in_m;
	pose.vecPosition[2] = position.position_y_in_m;

	pose.poseIsValid = true;
	pose.result = TrackingResult_Running_OK;
	pose.deviceIsConnected = true;

	HmdQuaternion_t quat;
	quat.w = position.orientation_w;
	quat.x = -position.orientation_x;
	quat.y = -position.orientation_y;
	quat.z = -position.orientation_z;

	pose.qWorldFromDriverRotation = quat;
	pose.qDriverFromHeadRotation = quat;

	return pose;
}

void ControllerDriver::RunFrame()
{
	////Since we used VRScalarUnits_NormalizedTwoSided as the unit, the range is -1 to 1.
	//VRDriverInput()->UpdateScalarComponent(joystickYHandle, 0.95f, 0); //move forward
	//VRDriverInput()->UpdateScalarComponent(trackpadYHandle, 0.95f, 0); //move foward
	//VRDriverInput()->UpdateScalarComponent(joystickXHandle, 0.0f, 0); //change the value to move sideways
	//VRDriverInput()->UpdateScalarComponent(trackpadXHandle, 0.0f, 0); //change the value to move sideways

	VRServerDriverHost()->TrackedDevicePoseUpdated(this->device_index_, GetPose(), sizeof(vr::DriverPose_t));
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
