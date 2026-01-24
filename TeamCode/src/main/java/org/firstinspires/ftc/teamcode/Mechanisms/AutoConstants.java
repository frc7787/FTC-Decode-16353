package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

@Configurable
public class AutoConstants {
    // INITIALIZING POSES

    public static Pose startPose = new Pose(28, 125, Math.toRadians(180)); // Start Pose of our robot.
    public static Pose leavePoseGoal = new Pose(55, 100, Math.toRadians(147)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static Pose startPoseAudience = new Pose(58, 9, Math.toRadians(90)); // Start Pose of our robot.x was 56
    //public static Pose leavePoseAudience = new Pose(50,26, Math.toRadians(114)); //
    public static Pose leavePoseAudience = new Pose(36,10, Math.toRadians(180));

    public static Pose pickup1StartPose = new Pose(48, 83, Math.toRadians(180));
    public static Pose pickup1EndPose = new Pose(18,83, Math.toRadians(180));


    public static Pose pickup2StartPrePose = new Pose(52,57, Math.toRadians(180));
    public static Pose pickup2StartPose = new Pose(48,57, Math.toRadians(180));

    //public static Pose pickup2EndPose = new Pose(21,57,Math.toRadians(180));
    public static Pose pickup2EndPose = new Pose(11,57,Math.toRadians(180));

    public static Pose pickup3StartPose = new Pose(48, 34, Math.toRadians(180));
    //public static Pose pickup3EndPose = new Pose(21,34, Math.toRadians(180));
    public static Pose pickup3EndPose = new Pose(11,34, Math.toRadians(180));

    // THIS scorePose is different from the mirror image of Red Goal in AutoConstants Red
    //public static Pose scorePose = new Pose(56, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    // USE THIS scorePose to have the mirror image of Red Goal
    public static Pose scorePose = new Pose(61, 94, Math.toRadians(138)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseFake = new Pose(56, 85, Math.toRadians(136)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    // let's move the score pose for Pickup 2 OFF the launch line
    public static Pose scorePoseFake2 = new Pose(57, 97, Math.toRadians(147)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static Pose scorePoseAudience = new Pose(56,21, Math.toRadians(112)); // Scoring Pose from the Audience launch zone.
    public static Pose scorePoseAudienceFake = new Pose(55,21, Math.toRadians(115)); // Scoring Pose from the Audience launch zone.


    public static Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    public static Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    // INITIALIZING PATHS
    public static Path scorePreload, scorePreloadAudience;
    public static PathChain grabPickup1, scorePickup1, grabPickup2Pre, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    public static PathChain grabPickup3Audience, scorePickup3Audience, grabPickup2PreAudience, grabPickup2Audience, scorePickup2Audience;
    public static PathChain leaveGoal, leaveAudience;
}
