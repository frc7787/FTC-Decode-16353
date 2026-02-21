package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

@Configurable
public class AutoConstantsRed {
    // INITIALIZING POSES

    public static Pose startPose = new Pose(116, 125, Math.toRadians(0)); // GOAL Start Pose of our robot.
    public static Pose leavePoseGoal = new Pose(96, 80, Math.toRadians(45)); // GOAL Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static Pose startPoseAudience = new Pose(88, 9, Math.toRadians(90)); // Start Pose of our robot.
    public static Pose leavePoseAudience = new Pose(108,10, Math.toRadians(0)); // was Pose(94,26, Math.toRadians(66))

    public static Pose pickup1StartPose = new Pose(96, 83, Math.toRadians(0));
    public static Pose pickup1EndPose = new Pose(124,83, Math.toRadians(0));


    public static Pose pickup2StartPrePose = new Pose(92,57, Math.toRadians(0));
    public static Pose pickup2StartPose = new Pose(96,57, Math.toRadians(0));

    public static Pose pickup2EndPose = new Pose(133.5,57,Math.toRadians(0));

    public static Pose pickup3StartPose = new Pose(96, 34, Math.toRadians(0));
    public static Pose pickup3EndPose = new Pose(134,34, Math.toRadians(0));

    public static Pose pickupBallsPose = new Pose(134,10,0); // steal enemy balls!

    //public static Pose scorePose = new Pose(88, 85, Math.toRadians(45)); // GOAL Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static double scorePoseHeading = 35; //45
    public static Pose scorePose = new Pose(88, 105, Math.toRadians(scorePoseHeading)); // (86,94), 43 GOAL Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    // if the stupid thing keeps going to the wrong scorePose, just make a new one ...
    public static double scorePoseFakeHeading = 32; //41
    public static Pose scorePoseFake = new Pose(88, 105, Math.toRadians(scorePoseFakeHeading)); // (88,92),40 GOAL Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static double scorePoseFakeHeading2 = 32; //41
    public static Pose scorePoseFake2 = new Pose(88, 105, Math.toRadians(scorePoseFakeHeading2)); // (86,100),38 GOAL Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseFake2Control = new Pose(83,58); // (83,58);


    //public static Pose scorePoseAudience = new Pose(89,21, Math.toRadians(67)); // Scoring Pose from the Audience launch zone.
    public static double scorePoseAudienceHeading = 70;
    public static Pose scorePoseAudience = new Pose(88,21, Math.toRadians(scorePoseAudienceHeading)); // (85,19,68) 88,15,72Scoring Pose from the Audience launch zone.
    public static Pose scorePoseAudienceBalls = new Pose(88,15, Math.toRadians(78)); // Scoring Pose from the Audience launch zone.

    public static double scorePoseAudienceFakeHeading = 69;
    public static Pose scorePoseAudienceFake = new Pose(88, 15, Math.toRadians(scorePoseAudienceFakeHeading));

    public static Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    public static Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    // INITIALIZING PATHS
    public static Path scorePreload, scorePreloadAudience;
    public static PathChain grabPickup1, scorePickup1, grabPickup2Pre, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    public static PathChain grabPickup3Audience, scorePickup3Audience, grabPickup2PreAudience, grabPickup2Audience, scorePickup2Audience, pickupBalls, leaveBalls, scorePickupBalls;
    public static PathChain leaveGoal, leaveAudience;
}
