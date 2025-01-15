#include "PoseHelper.hpp"
#include <cmath>

AmfitrackDriver::HmdQuaternion_t AmfitrackDriver::PoseHelper::HmdQuaternion_Normalize(const HmdQuaternion_t &q)
{
    HmdQuaternion_t result{};
    double n = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);

    if (n == 0.0)
    {
        return result; // Return the default zero quaternion if magnitude is zero
    }

    result.w = q.w / n;
    result.x = q.x / n;
    result.y = q.y / n;
    result.z = q.z / n;

    return result;
}

// Utility function for quaternion multiplication
AmfitrackDriver::HmdQuaternion_t AmfitrackDriver::PoseHelper::MultiplyQuaternions(const HmdQuaternion_t &q1, const HmdQuaternion_t &q2)
{
    HmdQuaternion_t result;
    result.w = q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z;
    result.x = q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y;
    result.y = q1.w * q2.y - q1.x * q2.z + q1.y * q2.w + q1.z * q2.x;
    result.z = q1.w * q2.z + q1.x * q2.y - q1.y * q2.x + q1.z * q2.w;
    return result;
}

AmfitrackDriver::VRPose AmfitrackDriver::PoseHelper::GetSourcePose(VRPose &generic_tracker_pose, VRPose &tracker_pose_in_steamvr)
{
    // Convert generic tracker position from Amfitrack to SteamVR space (flip Y, Z)
    AmfitrackDriver::HmdVector3_t amfitrack_tracker_position{};
    amfitrack_tracker_position.v[0] = generic_tracker_pose.Position.v[0];
    amfitrack_tracker_position.v[1] = -generic_tracker_pose.Position.v[1]; // Flip Y
    amfitrack_tracker_position.v[2] = -generic_tracker_pose.Position.v[2]; // Flip Z

    // Invert the Amfitrack tracker position
    AmfitrackDriver::HmdVector3_t inverse_amfitrack_tracker_position{};
    inverse_amfitrack_tracker_position.v[0] = -amfitrack_tracker_position.v[0];
    inverse_amfitrack_tracker_position.v[1] = -amfitrack_tracker_position.v[1];
    inverse_amfitrack_tracker_position.v[2] = -amfitrack_tracker_position.v[2];

    // Get the tracker's position in SteamVR
    AmfitrackDriver::HmdVector3_t tracker_position = tracker_pose_in_steamvr.Position;

    // Compute the source position in SteamVR space
    AmfitrackDriver::VRPose steamVR_source_position = {};
    steamVR_source_position.Position.v[0] = tracker_position.v[0] + inverse_amfitrack_tracker_position.v[0];
    steamVR_source_position.Position.v[1] = tracker_position.v[1] + inverse_amfitrack_tracker_position.v[1];
    steamVR_source_position.Position.v[2] = tracker_position.v[2] + inverse_amfitrack_tracker_position.v[2];

    return steamVR_source_position;
}

AmfitrackDriver::VRPose AmfitrackDriver::PoseHelper::CalculateControllerPose(VRPose &steamVR_source_position, VRPose &controller_pose)
{
    VRPose out_pose{};
    // Convert the controller's Amfitrack position to SteamVR coordinates (flip Y, Z)
    HmdVector3_t amfitrack_controller_position{};
    amfitrack_controller_position.v[0] = controller_pose.Position.v[0];
    amfitrack_controller_position.v[1] = -controller_pose.Position.v[1]; // Flip Y-axis
    amfitrack_controller_position.v[2] = -controller_pose.Position.v[2]; // Flip Z-axis

    // Compute the controller's position in SteamVR space
    HmdVector3_t controller_position_in_vr_space{};
    controller_position_in_vr_space.v[0] = steamVR_source_position.Position.v[0] + amfitrack_controller_position.v[0];
    controller_position_in_vr_space.v[1] = steamVR_source_position.Position.v[1] + amfitrack_controller_position.v[1];
    controller_position_in_vr_space.v[2] = steamVR_source_position.Position.v[2] + amfitrack_controller_position.v[2];

    out_pose.Position.v[0] = controller_position_in_vr_space.v[0];
    out_pose.Position.v[1] = controller_position_in_vr_space.v[1];
    out_pose.Position.v[2] = controller_position_in_vr_space.v[2];

    // Convert the controller's orientation from Amfitrack to SteamVR by flipping Y and Z axes
    HmdQuaternion_t amfitrack_rotation{};
    amfitrack_rotation.w = controller_pose.Orientation.w;
    amfitrack_rotation.x = controller_pose.Orientation.x;
    amfitrack_rotation.y = -controller_pose.Orientation.y;
    amfitrack_rotation.z = -controller_pose.Orientation.z;

    // Apply rotation adjustments
    HmdQuaternion_t rotationQuatY = {0.7071068, 0, 0.7071068, 0}; // 90° Y rotation
    HmdQuaternion_t rotationQuatX = {0.7071068, 0, 0, 0.7071068}; // 90° X rotation
    HmdQuaternion_t rotationQuatZ = {0, 1, 0, 0};                 // 180° Z rotation

    // Combine rotations and normalize
    HmdQuaternion_t corrected_rotation = MultiplyQuaternions(amfitrack_rotation, rotationQuatY);
    corrected_rotation = MultiplyQuaternions(corrected_rotation, rotationQuatX);
    corrected_rotation = MultiplyQuaternions(corrected_rotation, rotationQuatZ);

    // Normalize final quaternion
    corrected_rotation = HmdQuaternion_Normalize(corrected_rotation);
    out_pose.Orientation = corrected_rotation;

    // Mark pose as valid
    out_pose.PoseCorrect = true;

    return out_pose;
};
