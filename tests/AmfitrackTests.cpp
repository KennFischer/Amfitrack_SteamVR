#include <gtest/gtest.h>
#include "PoseHelper.hpp"

AmfitrackDriver::PoseHelper poseHelper;

TEST(GetSourcePoseTest, ScenarioA)
{
    // 2) Create mock inputs
    AmfitrackDriver::VRPose generic_pose;
    generic_pose.Position.v[0] = -0.039050f;
    generic_pose.Position.v[1] = 0.493840f;
    generic_pose.Position.v[2] = 0.737350f;

    // 3) Create a mock tracker pose in SteamVR.
    AmfitrackDriver::VRPose tracker_pose_in_steamvr;

    tracker_pose_in_steamvr.Position.v[0] = 0.110180f;  // X
    tracker_pose_in_steamvr.Position.v[1] = 0.838261f;  // Y
    tracker_pose_in_steamvr.Position.v[2] = -0.351854f; // Z

    AmfitrackDriver::VRPose result = poseHelper.GetSourcePose(generic_pose, tracker_pose_in_steamvr);

    AmfitrackDriver::HmdVector3_t expected = {0.149230f, 1.332101f, 0.385496f};

    // 6) Assert
    EXPECT_NEAR(result.Position.v[0], expected.v[0], 0.0001f);
    EXPECT_NEAR(result.Position.v[1], expected.v[1], 0.0001f);
    EXPECT_NEAR(result.Position.v[2], expected.v[2], 0.0001f);
}

TEST(GetSourcePoseTest, ScenarioB)
{
    // 2) Create mock inputs from your logs
    AmfitrackDriver::VRPose generic_pose;
    generic_pose.Position.v[0] = 0.025140f;
    generic_pose.Position.v[1] = -0.471000f;
    generic_pose.Position.v[2] = -0.716100f;
    // Orientation not used by GetSourcePose

    // 3) Create a mock tracker pose in SteamVR
    AmfitrackDriver::VRPose tracker_pose_in_steamvr;

    tracker_pose_in_steamvr.Position.v[0] = 0.156554f;  // X
    tracker_pose_in_steamvr.Position.v[1] = 0.824118f;  // Y
    tracker_pose_in_steamvr.Position.v[2] = -0.322424f; // Z

    AmfitrackDriver::VRPose result = poseHelper.GetSourcePose(generic_pose, tracker_pose_in_steamvr);

    AmfitrackDriver::HmdVector3_t expected = {0.131414f, 1.295118f, 0.393676f};

    // 6) Assertions
    EXPECT_NEAR(result.Position.v[0], expected.v[0], 0.0001f);
    EXPECT_NEAR(result.Position.v[1], expected.v[1], 0.0001f);
    EXPECT_NEAR(result.Position.v[2], expected.v[2], 0.0001f);
}

TEST(GetSourcePoseTest, ScenarioC)
{
    // 2) Mock inputs
    AmfitrackDriver::VRPose generic_pose;
    generic_pose.Position.v[0] = -0.610330f;
    generic_pose.Position.v[1] = 0.533400f;
    generic_pose.Position.v[2] = 0.369150f;

    // 3) Mock tracker pose (3×4 matrix)
    AmfitrackDriver::VRPose tracker_pose_in_steamvr;
    tracker_pose_in_steamvr.Position.v[0] = -0.457048f; // X
    tracker_pose_in_steamvr.Position.v[1] = 0.831017f;  // Y
    tracker_pose_in_steamvr.Position.v[2] = -0.194010f; // Z

    AmfitrackDriver::VRPose result = poseHelper.GetSourcePose(generic_pose, tracker_pose_in_steamvr);

    AmfitrackDriver::VRPose controller_pose{};
    controller_pose.Orientation = {
        1,
        0,
        0,
        0};

    controller_pose.Position = {
        -0.805280,
        0.774210,
        0.409140};

    // --- Calculate the controller's pose in SteamVR space ---
    AmfitrackDriver::VRPose resultpose = poseHelper.CalculateControllerPose(result, controller_pose);

    AmfitrackDriver::HmdVector3_t expected = {-0.651998, 0.590207, -0.234000};

    // 6) Assertions
    EXPECT_NEAR(resultpose.Position.v[0], expected.v[0], 0.0001f);
    EXPECT_NEAR(resultpose.Position.v[1], expected.v[1], 0.0001f);
    EXPECT_NEAR(resultpose.Position.v[2], expected.v[2], 0.0001f);
}