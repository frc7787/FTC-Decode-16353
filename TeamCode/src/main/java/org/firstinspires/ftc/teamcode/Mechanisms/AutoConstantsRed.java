package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

public class AutoConstantsRed {
    // INITIALIZING POSES

    public static Pose startPose = new Pose(116, 127, Math.toRadians(0)); // Start Pose of our robot.
    public static Pose leavePoseGoal = new Pose(96, 80, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static Pose startPoseAudience = new Pose(88, 9, Math.toRadians(90)); // Start Pose of our robot.
    public static Pose leavePoseAudience = new Pose(94,26, Math.toRadians(66)); //

    public static Pose pickup1StartPose = new Pose(96, 83, Math.toRadians(0));
    public static Pose pickup1EndPose = new Pose(120,83, Math.toRadians(0));


    public static Pose pickup2StartPrePose = new Pose(92,60, Math.toRadians(0));
    public static Pose pickup2StartPose = new Pose(96,60, Math.toRadians(0));

    public static Pose pickup2EndPose = new Pose(125,60,Math.toRadians(0));

    public static Pose pickup3StartPose = new Pose(96, 34, Math.toRadians(0));
    public static Pose pickup3EndPose = new Pose(123,34, Math.toRadians(0));
    public static Pose scorePose = new Pose(88, 85, Math.toRadians(45)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseAudience = new Pose(89,21, Math.toRadians(67)); // Scoring Pose from the Audience launch zone.
    public static Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    public static Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    // INITIALIZING PATHS
    public static Path scorePreload, scorePreloadAudience;
    public static PathChain grabPickup1, scorePickup1, grabPickup2Pre, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    public static PathChain grabPickup3Audience, scorePickup3Audience, grabPickup2PreAudience, grabPickup2Audience, scorePickup2Audience;
    public static PathChain leaveGoal, leaveAudience;
}
