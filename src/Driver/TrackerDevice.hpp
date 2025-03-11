#pragma once

#include <chrono>
#include <cmath>

#include <linalg.h>

#include <Driver/IVRDevice.hpp>
#include <Native/DriverFactory.hpp>
#include <openvr_driver.h>

#include "PoseHelper.hpp"

namespace AmfitrackDriver {
    class TrackerDevice : public IVRDevice {
    public:
        TrackerDevice(uint8_t deviceId, std::string serial);
        ~TrackerDevice() = default;

        // Inherited via IVRDevice
        virtual std::string GetSerial() override;
        virtual void Update() override;
        virtual vr::TrackedDeviceIndex_t GetDeviceIndex() override;
        virtual DeviceType GetDeviceType() override;

        virtual vr::EVRInitError Activate(uint32_t unObjectId) override;
        virtual void Deactivate() override;
        virtual void EnterStandby() override;
        virtual void* GetComponent(const char* pchComponentNameAndVersion) override;
        virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override;
        virtual vr::DriverPose_t GetPose() override;
        void RegisterButtonPress(uint16_t gpio_state);

        vr::DriverPose_t GetLastPose() const { return last_pose_; } 

    private:
        vr::TrackedDeviceIndex_t device_index_ = vr::k_unTrackedDeviceIndexInvalid;
        std::string serial_;
        uint8_t deviceID_;

        vr::DriverPose_t last_pose_ = IVRDevice::MakeDefaultPose();

        bool did_vibrate_ = false;
        float vibrate_anim_state_ = 0.f;

        vr::VRInputComponentHandle_t haptic_component_ = 0;

        vr::VRInputComponentHandle_t system_click_component_ = 0;
        vr::VRInputComponentHandle_t system_touch_component_ = 0;

        vr::DriverPose_t ToDriverPose(AmfitrackDriver::VRPose &pose);

    };
};
