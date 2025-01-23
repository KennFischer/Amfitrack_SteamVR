#pragma once

namespace AmfitrackDriver
{
    struct HmdVector3_t
    {
        float v[3];
    };
    struct HmdQuaternion_t
    {
        double w, x, y, z;
    };
    struct VRPose
    {
        int OrientationRealWorld; // 1-6 depending on how it is defined
        bool PoseCorrect;
        HmdVector3_t Position;
        HmdQuaternion_t Orientation;
    };
    class PoseHelper
    {
    public:
        static VRPose GetSourcePose(VRPose &generic_tracker_pose, VRPose &tracker_pose_in_steamvr);
        static VRPose CalculateControllerPose(VRPose &steamVR_source_position, VRPose &controller_pose);
        static HmdQuaternion_t MultiplyQuaternions(const HmdQuaternion_t &q1, const HmdQuaternion_t &q2);
        static HmdQuaternion_t HmdQuaternion_Normalize(const HmdQuaternion_t &q);

    private:
    };
};
