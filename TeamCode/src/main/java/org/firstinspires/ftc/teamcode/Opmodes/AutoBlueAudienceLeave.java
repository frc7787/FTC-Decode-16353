package org.firstinspires.ftc.teamcode.Opmodes;


import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import static org.firstinspires.ftc.teamcode.Mechanisms.AutoConstants.*;

@Autonomous(name = "AutoBlueAudienceLeave", group = "opmodes")
public class AutoBlueAudienceLeave extends  OpMode{

    private Intake intake;
    private Shooter shooter;

    private AprilTagSubsystem aprilTagSubsystem;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    // INITIALIZING POSES

    /*

    private final Pose startPose = new Pose(28, 127, Math.toRadians(180)); // Start Pose of our robot.
    private final Pose leavePoseGoal = new Pose(60, 65, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.

    private final Pose startPoseAudience = new Pose(56, 9, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose leavePoseAudience = new Pose(50,26, Math.toRadians(114)); //

    private final Pose pickup1StartPose = new Pose(48, 83, Math.toRadians(180));
    private final Pose pickup1EndPose = new Pose(24,83, Math.toRadians(180));


    private final Pose pickup2StartPrePose = new Pose(52,58, Math.toRadians(180));
    private final Pose pickup2StartPose = new Pose(48,58, Math.toRadians(180));

    private final Pose pickup2EndPose = new Pose(20,58,Math.toRadians(180));

    private final Pose pickup3StartPose = new Pose(48, 34, Math.toRadians(180));
    private final Pose pickup3EndPose = new Pose(20,34, Math.toRadians(180));
    private final Pose scorePose = new Pose(60, 85, Math.toRadians(135)); // Scoring Pose of our robot. It is facing the goal at a 135 degree angle.
    private final Pose scorePoseAudience = new Pose(55,21, Math.toRadians(113)); // Scoring Pose from the Audience launch zone.
    private final Pose pickup1Pose = new Pose(37, 121, Math.toRadians(0)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(43, 130, Math.toRadians(0)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose pickup3Pose = new Pose(49, 135, Math.toRadians(0)); // Lowest (Third Set) of Artifacts from the Spike Mark.

     */

    // INITIALIZING PATHS

    /*
    private Path scorePreload, scorePreloadAudience;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3;
    private PathChain grabPickup3Audience, scorePickup3Audience, grabPickup2PreAudience, grabPickup2Audience, scorePickup2Audience;
    private PathChain leaveGoal, leaveAudience;

     */

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1StartPose.getHeading())
                .addPath(new BezierLine(pickup1StartPose, pickup1EndPose))
                .setLinearHeadingInterpolation(pickup1StartPose.getHeading(), pickup1EndPose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2StartPose.getHeading())
                .addPath(new BezierLine(pickup2StartPose,pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2StartPose.getHeading(),pickup2EndPose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, scorePose))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup3StartPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3StartPose.getHeading())
                .build();

        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        leaveGoal = follower.pathBuilder()
                .addPath(new BezierLine(scorePose,leavePoseGoal))
                .setLinearHeadingInterpolation(scorePose.getHeading(),leavePoseGoal.getHeading())
                .build();

        // AUDIENCE SIDE PATHS

        scorePreloadAudience = new Path(new BezierLine(startPoseAudience, scorePoseAudience));
        scorePreloadAudience.setLinearHeadingInterpolation(startPoseAudience.getHeading(), scorePoseAudience.getHeading());

        grabPickup3Audience = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseAudience, pickup3StartPose))
                .setLinearHeadingInterpolation(scorePoseAudience.getHeading(), pickup3StartPose.getHeading())
                .addPath(new BezierLine(pickup3StartPose, pickup3EndPose))
                .setLinearHeadingInterpolation(pickup3StartPose.getHeading(), pickup3EndPose.getHeading())
                .setGlobalDeceleration(4)
                .setBrakingStrength(4)
                .setBrakingStart(4)
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3Audience= follower.pathBuilder()
                .addPath(new BezierLine(pickup3EndPose, scorePoseAudience))
                .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePoseAudience.getHeading())
                .setGlobalDeceleration(4)
                .setBrakingStrength(4)
                .setBrakingStart(4)
                .build();

        grabPickup2PreAudience = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseAudience, pickup2StartPrePose))
                .setLinearHeadingInterpolation(scorePoseAudience.getHeading(), pickup2StartPrePose.getHeading())
                .addPath(new BezierLine(pickup2StartPrePose, pickup2StartPose))
                .setLinearHeadingInterpolation(pickup2StartPrePose.getHeading(), pickup2StartPose.getHeading())
                .build();

        grabPickup2Audience = follower.pathBuilder()
                .addPath(new BezierLine(pickup2StartPose, pickup2EndPose))
                .setLinearHeadingInterpolation(pickup2StartPose.getHeading(), pickup2EndPose.getHeading())
                .setGlobalDeceleration(3)
                .setBrakingStrength(4)
                .setBrakingStart(4)
                .build();

        scorePickup2Audience= follower.pathBuilder()
                .addPath(new BezierLine(pickup2EndPose, scorePoseAudience))
                .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePoseAudience.getHeading())
                .setGlobalDeceleration(4)
                .setBrakingStrength(4)
                .setBrakingStart(4)
                .build();

        leaveAudience = follower.pathBuilder()
                .addPath(new BezierLine(scorePoseAudience,leavePoseAudience))
                .setLinearHeadingInterpolation(scorePoseAudience.getHeading(),leavePoseAudience.getHeading())
                .build();


    } // end of BuildPaths


    // MANAGING PATH STATES
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: {  // FOLLOW PATH TO SCORING - preloaded
                follower.followPath(scorePreloadAudience);
                setPathState(1);
                //shooter.spin(2000);
                break;
            }
            case 1: { // DONE PATH

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Score Preload */
                    setPathState(2);
                }
                break;
            }
            case 2: { // JUST SCORING - preloaded
                if (shooter.score(false, 3, telemetry)) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(leaveAudience, 0.5, true);
                    // setPathState(2); OK, let's just test the first two paths.
                    intake.spin(0.0);
                    setPathState(8); // go straight to LEAVE AUDIENCE
                }
                break;
            }
            case 3: { // FOLLOW GRAB PATH - pickup 3
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    intake.spin(1.0);
                    follower.followPath(scorePickup3Audience, true);
                    setPathState(4);
                }
                break;
            }
            case 4: { // FOLLOW PATH TO SCORING - pickup 3
                if (!follower.isBusy()) {
                    setPathState(5);
                }
                break;
            }
            case 5: { // JUST SCORING - pickup 3
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (shooter.score(false, 3, telemetry)) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup2PreAudience, true);
                    intake.spin(0.8);
                    setPathState(6);
                }
                break;
            }
            case 6: {
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2Audience,0.5,true);
                    setPathState(61);
                }
                break;
            }


            case 61: { // FOLLOW GRAB PATH - pickup 2
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    intake.spin(0.8);
                    follower.followPath(scorePickup2Audience, true);
                    setPathState(7);
                }
                break;
            }
            case 7: { // FOLLOW PATH TO SCORING - pickup 2
                if (!follower.isBusy()) {
                    setPathState(8);
                }
                break;
            }
            case 8: { // JUST SCORING - pickup 2
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (shooter.score(false, 3, telemetry)) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    //follower.followPath(grabPickup3,true);
                    follower.followPath(leaveAudience, true);
                    intake.spin(0.0);  // POWER DOWN FOR END OF AUTO
                    shooter.spin(0);
                    setPathState(9);
                }
                break;
            }
            case 9: { // FOLLOW PATH LEAVEAUDIENCE
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
            }
            case 66:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 77:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    } // end autonomousPathUpdate

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    } // end setPathState

    // INIT AND LOOP METHODS
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    } // end loop

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPoseAudience);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);

    } // end init

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}




} // end of AutoByExampleDec
