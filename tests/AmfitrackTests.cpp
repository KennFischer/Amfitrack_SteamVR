#include <gtest/gtest.h>
#include "PoseHelper.hpp"

AmfitrackDriver::PoseHelper poseHelper;

float input_positions[][3] = {
    {-0.039050, 0.493840, 0.737350},
    {-0.039300, 0.491330, 0.732520},
    {0.025140, 0.471000, 0.716100},
    {-0.096080, 0.481050, 0.695530},
    {-0.245540, 0.499570, 0.708280},
    {-0.346560, 0.491420, 0.646270},
    {-0.446730, 0.504120, 0.538180},
    {-0.610330, 0.533400, 0.369150},
    {-0.582320, 0.514320, 0.415470},
    {-0.634250, 0.514270, 0.278750},
    {-0.714430, 0.533260, -0.028900},
    {-0.717740, 0.522530, 0.054820},
    {-0.696900, 0.521110, -0.085890},
    {-0.501040, 0.547650, -0.442020},
    {-0.540900, 0.524750, -0.392640},
    {-0.407870, 0.532320, -0.475770},
    {-0.014810, 0.534880, -0.451050},
    {-0.067260, 0.519330, -0.433630},
    {0.065070, 0.522310, -0.451810},
    {0.574980, 0.534930, -0.368970},
    {0.554860, 0.523000, -0.412360},
    {0.650290, 0.517500, -0.292630},
    {0.697240, 0.521820, -0.010770}
};

float tracker_positions[][3] = {
    {0.110180, 0.838261, -0.351854},
    {0.102837, 0.834072, -0.348367},
    {0.156554, 0.824118, -0.322424},
    {0.043736, 0.864425, -0.324883},
    {-0.092458, 0.832229, -0.379381},
    {-0.182578, 0.824267, -0.354586},
    {-0.273540, 0.865035, -0.286395},
    {-0.457048, 0.831017, -0.194010},
    {-0.429616, 0.822991, -0.222547},
    {-0.482707, 0.862952, -0.117057},
    {-0.625155, 0.832569, 0.111478},
    {-0.616324, 0.821078, 0.038541},
    {-0.618192, 0.861025, 0.171045},
    {-0.536052, 0.822394, 0.516940},
    {-0.563946, 0.813664, 0.467660},
    {-0.467018, 0.854912, 0.556994},
    {-0.122215, 0.821802, 0.598213},
    {-0.173851, 0.812830, 0.575844},
    {-0.051691, 0.855622, 0.613436},
    {0.406643, 0.826118, 0.681157},
    {0.348180, 0.816041, 0.709261},
    {0.444382, 0.857570, 0.630515},
    {0.540661, 0.828543, 0.436974}
};

float controller_poses[][3] = {
    {-0.060610, 0.722610, 0.911690},
    {-0.058920, 0.726020, 0.911080},
    {-0.058640, 0.721440, 0.909880},
    {-0.064050, 0.721270, 0.911970},
    {-0.355440, 0.737280, 0.851610},
    {-0.554170, 0.753600, 0.729360},
    {-0.550440, 0.751790, 0.733530},
    {-0.805280, 0.774210, 0.409140},
    {-0.805540, 0.774200, 0.403210},
    {-0.804010, 0.776600, 0.407080},
    {-0.904370, 0.777560, -0.082030},
    {-0.900550, 0.782950, -0.071550},
    {-0.902400, 0.780750, -0.083610},
    {-0.598710, 0.792540, -0.602410},
    {-0.596530, 0.792080, -0.603430},
    {-0.599390, 0.789170, -0.600780},
    {0.020230, 0.783970, -0.649980},
    {0.028170, 0.785610, -0.649220},
    {0.029750, 0.786690, -0.647720},
    {0.728940, 0.772310, -0.483890},
    {0.777030, 0.778720, -0.448360},
    {0.777360, 0.773810, -0.455880},
    {0.893320, 0.756080, 0.054390}
};

float final_positions[][3] = {
    {0.088620, 0.609491, -0.526194},
    {0.083217, 0.599382, -0.526927},
    {0.072774, 0.573678, -0.516204},
    {0.075766, 0.624205, -0.541323},
    {-0.202358, 0.594519, -0.522711},
    {-0.390188, 0.562087, -0.437676},
    {-0.377250, 0.617364, -0.481745},
    {-0.651998, 0.590207, -0.234000},
    {-0.652836, 0.563111, -0.210287},
    {-0.652467, 0.600622, -0.245387},
    {-0.815095, 0.588269, 0.164608},
    {-0.799134, 0.560658, 0.164911},
    {-0.823692, 0.601385, 0.168765},
    {-0.633722, 0.577504, 0.677330},
    {-0.619576, 0.546334, 0.678450},
    {-0.658538, 0.598063, 0.682004},
    {-0.087175, 0.572712, 0.797143},
    {-0.078421, 0.546550, 0.791434},
    {-0.087011, 0.591242, 0.809346},
    {0.560603, 0.588738, 0.796077},
    {0.570350, 0.560320, 0.745261},
    {0.571452, 0.601260, 0.793765},
    {0.736741, 0.594283, 0.371814}
};

TEST(GetSourcePoseTest, ScenarioA) 
{
    // 2) Create mock inputs
    AmfitrackDriver::VRPose generic_pose;
    generic_pose.Position.v[0] = -0.222820f;
    generic_pose.Position.v[1] = 0.774140f;
    generic_pose.Position.v[2] = 0.950350f;

    // 3) Create a mock tracker pose in SteamVR.
    AmfitrackDriver::VRPose tracker_pose_in_steamvr;

    tracker_pose_in_steamvr.Position.v[0] = 0.125000f;  // X
    tracker_pose_in_steamvr.Position.v[1] = 0.899440f;  // Y
    tracker_pose_in_steamvr.Position.v[2] = -0.475175f; // Z

    AmfitrackDriver::VRPose result = poseHelper.GetSourcePose(generic_pose, tracker_pose_in_steamvr);

    AmfitrackDriver::HmdVector3_t expected = {0.347820f, 1.673580f, 0.475175f};

    // 6) Assert
    EXPECT_NEAR(result.Position.v[0], expected.v[0], 0.0001f);
    EXPECT_NEAR(result.Position.v[1], expected.v[1], 0.0001f);
    EXPECT_NEAR(result.Position.v[2], expected.v[2], 0.0001f);
}

 TEST(GetSourcePoseTest, ScenarioB)
 {
     // 2) Create mock inputs from your logs
         AmfitrackDriver::VRPose generic_pose;
     generic_pose.Position.v[0] =  0.025140f;
     generic_pose.Position.v[1] = 0.471000f;
     generic_pose.Position.v[2] = 0.716100f;
     // Orientation not used by GetSourcePose

     // 3) Create a mock tracker pose in SteamVR
     AmfitrackDriver::VRPose tracker_pose_in_steamvr;

     tracker_pose_in_steamvr.Position.v[0] =  0.156554f;  // X
     tracker_pose_in_steamvr.Position.v[1] =  0.824118f;  // Y
     tracker_pose_in_steamvr.Position.v[2] = -0.322424f;  // Z

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
    tracker_pose_in_steamvr.Position.v[1] =  0.831017f; // Y
    tracker_pose_in_steamvr.Position.v[2] = -0.194010f; // Z

    AmfitrackDriver::VRPose result = poseHelper.GetSourcePose(generic_pose, tracker_pose_in_steamvr);

    AmfitrackDriver::VRPose controller_pose{};
    controller_pose.Orientation = {
        1,
        0,
        0,
        0 };

    controller_pose.Position = {
        -0.805280,
        0.774210,
        0.409140 };

    // --- Calculate the controller's pose in SteamVR space ---
    AmfitrackDriver::VRPose resultpose = poseHelper.CalculateControllerPose(result, controller_pose);

    AmfitrackDriver::HmdVector3_t expected = { -0.651998, 0.590207, -0.234000 };

    // 6) Assertions
    EXPECT_NEAR(resultpose.Position.v[0], expected.v[0], 0.0001f);
    EXPECT_NEAR(resultpose.Position.v[1], expected.v[1], 0.0001f);
    EXPECT_NEAR(resultpose.Position.v[2], expected.v[2], 0.0001f);
}

TEST(GetSourcePoseTest, ScenarioD)
{
    for (int i = 0; i < 23; i++)
    {
        AmfitrackDriver::VRPose generic_pose;
        generic_pose.Position.v[0] = input_positions[i][0];
        generic_pose.Position.v[1] = input_positions[i][1];
        generic_pose.Position.v[2] = input_positions[i][2];


        // 3) Mock tracker pose (3×4 matrix)
        AmfitrackDriver::VRPose tracker_pose_in_steamvr;

        tracker_pose_in_steamvr.Position.v[0] = tracker_positions[i][0]; // X
        tracker_pose_in_steamvr.Position.v[1] = tracker_positions[i][1]; // Y
        tracker_pose_in_steamvr.Position.v[2] = tracker_positions[i][2]; // Z

        AmfitrackDriver::VRPose result = poseHelper.GetSourcePose(generic_pose, tracker_pose_in_steamvr);

        AmfitrackDriver::VRPose controller_pose{};
        controller_pose.Orientation = {
            1,
            0,
            0,
            0 };

        controller_pose.Position = {
            controller_poses[i][0],
            controller_poses[i][1],
            controller_poses[i][2] };

        // --- Calculate the controller's pose in SteamVR space ---
        AmfitrackDriver::VRPose resultpose = poseHelper.CalculateControllerPose(result, controller_pose);

        AmfitrackDriver::HmdVector3_t expected = { final_positions[i][0], final_positions[i][1], final_positions[i][2] };

        // 6) Assertions
        EXPECT_NEAR(resultpose.Position.v[0], expected.v[0], 0.0001f);
        EXPECT_NEAR(resultpose.Position.v[1], expected.v[1], 0.0001f);
        EXPECT_NEAR(resultpose.Position.v[2], expected.v[2], 0.0001f);
    }
}
