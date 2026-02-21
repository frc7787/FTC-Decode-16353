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
    public static Pose pickup2EndPose = new Pose(10,57,Math.toRadians(180));

    public static Pose pickup3StartPose = new Pose(48, 34, Math.toRadians(180));
    //public static Pose pickup3EndPose = new Pose(21,34, Math.toRadians(180));
    public static Pose pickup3EndPose = new Pose(10,34, Math.toRadians(180));

    public static Pose pickupBallsPose = new Pose(11,11,270); // 180 steal enemy balls!
    public static Pose pickupBallsPose2 = new Pose(11,8,270); // 195 steal enemy balls!


    // THIS scorePose is different from the mirror image of Red Goal in AutoConstants Red
    //public static Pose scorePose = new Pose(56, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    // USE THIS scorePose to have the mirror image of Red Goal
    // changed feb 18 public static Pose scorePose = new Pose(61, 94, Math.toRadians(138)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    // find new closer shot spot public static Pose scorePose = new Pose(58, 94, Math.toRadians(137)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static Pose scorePose = new Pose(56, 105, Math.toRadians(145)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    // changed feb 18 public static Pose scorePoseFake = new Pose(56, 85, Math.toRadians(136)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    // move closer public static Pose scorePoseFake = new Pose(56, 85, Math.toRadians(140)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    public static Pose scorePoseFake = new Pose(56, 105, Math.toRadians(148)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    // let's move the score pose for Pickup 2 OFF the launch line

    // changed feb 18 public static Pose scorePoseFake2 = new Pose(57, 97, Math.toRadians(147)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    //public static Pose scorePoseFake2 = new Pose(56, 99, Math.toRadians(142)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    public static Pose scorePoseFake2 = new Pose(56, 105, Math.toRadians(148)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.


    public static Pose scorePoseFake2Control = new Pose(61,58); // (61,58)
    public static Pose scorePoseDump = new Pose(56, 105, Math.toRadians(151));


    public static Pose scorePoseAudience = new Pose(57,21, Math.toRadians(110)); // was 112 Scoring Pose from the Audience launch zone.
    public static Pose scorePoseAudienceFake = new Pose(57,21, Math.toRadians(114)); // (55,21,110)was 115 Scoring Pose from the Audience launch zone.

    public static Pose dumpPose = new Pose(15,62, Math.toRadians(270));
    public static Pose dumpControl = new Pose(19, 59); // (37,63)

    public static Pose dumpScoreControl = new Pose(72,60); // (61,61)


    public static Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    public static Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    public static Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

    // INITIALIZING PATHS
    public static Path scorePreload, scorePreloadAudience;
    public static PathChain grabPickup1, scorePickup1, grabPickup2Pre, grabPickup2, scorePickup2, grabPickup3, scorePickup3, dumpPath, scoreDump;
    public static PathChain grabPickup3Audience, scorePickup3Audience, grabPickup2PreAudience, grabPickup2Audience, scorePickup2Audience, pickupBalls, pickupBalls2, leaveBalls, scorePickupBalls;
    public static PathChain leaveGoal, leaveAudience;
}
