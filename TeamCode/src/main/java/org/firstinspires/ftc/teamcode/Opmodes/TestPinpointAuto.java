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


@Autonomous(name = "Test Pinpoint Auto", group = "opmodes")
public class TestPinpointAuto extends  OpMode{

    private Intake intake;
    private Shooter shooter;

    private AprilTagSubsystem aprilTagSubsystem;

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private boolean DEBUG = true;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(90));
    private final Pose forwardPose = new Pose(0, 20, Math.toRadians(90));
    private final Pose backwardPose = new Pose(0, 5, Math.toRadians(90));
    private final Pose leftPose = new Pose(2, 5, Math.toRadians(180));
    private final Pose rightPose = new Pose(4, 5, Math.toRadians(90));

    public Path startPath;
    public PathChain backwardPath, rightPath, leftPath;

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

        startPath = new Path(new BezierLine(startPose, forwardPose));
        startPath.setLinearHeadingInterpolation(startPose.getHeading(), forwardPose.getHeading());

        backwardPath = follower.pathBuilder()
                .addPath(new BezierLine(forwardPose, backwardPose))
                .setLinearHeadingInterpolation(forwardPose.getHeading(), backwardPose.getHeading())
                .build();

        rightPath = follower.pathBuilder()
                .addPath(new BezierLine(backwardPose, rightPose))
                .setLinearHeadingInterpolation(backwardPose.getHeading(), rightPose.getHeading())
                .addPath(new BezierLine(rightPose, leftPose))
                .setLinearHeadingInterpolation(rightPose.getHeading(), leftPose.getHeading())
                .build();

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */


    } // end of BuildPaths


    // MANAGING PATH STATES
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: {  // FOLLOW PATH TO SCORING - preloaded
                follower.followPath(startPath, true);
                setPathState(1);
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
                    follower.followPath(backwardPath, 0.4, true);
                    setPathState(2);
                }
                break;
            }
            case 2: { // JUST SCORING - preloaded
                if (!follower.isBusy()) {
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(rightPath, 0.4, true);
                    setPathState(3);
                }
                break;
            }
            case 3: { // FOLLOW GRAB PATH - pickup 3
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    setPathState(4);
                }
                break;
            }
            case 4: { // FOLLOW PATH TO SCORING - pickup 3
                if (!follower.isBusy()) {
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
        } // end switch
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
        if (DEBUG) {
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        } // end DEBUG
    } // end loop

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);

        shooter.setShooterVelocity(shooter.RPM_AUDIENCE);

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


