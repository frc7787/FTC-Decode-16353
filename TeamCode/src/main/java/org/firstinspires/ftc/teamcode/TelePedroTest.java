package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.Flipper;
import org.firstinspires.ftc.teamcode.Mechanisms.IndicatorLights;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp(name = "TelePedroTest", group = "$")
public class TelePedroTest extends OpMode {

    // PEDRO PATHING STUFF
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private Supplier<PathChain> pathChainGoal;
    private TelemetryManager telemetryM;

    private AprilTagSubsystem aprilTagSubsystem;

    private MecanumDriveBase mecanumDrive;
    private Intake intake;
    private Shooter shooter;

    private Flipper flipper;

    private IndicatorLights indicatorLights;

    private boolean DEBUG = true;

    private double[] results;
    private double distanceAprilTag = 9999;
    private double angleAprilTag = 9999;
    private double shooterVelocity = 0;
    private double shooterTargetVelocity;
    private String shooterDistance = "Default";
    private enum AutomaticShooting {MANUAL, AUTO1,AUTO3};
    private AutomaticShooting automaticShooting = AutomaticShooting.MANUAL;

    private double[] idealRanges = {107,172,260,295};
    private double currentRange = 170;

    @Override public void init() {
        mecanumDrive = new MecanumDriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        flipper = new Flipper(hardwareMap);

        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);
        indicatorLights = new IndicatorLights(hardwareMap);

        shooterTargetVelocity = 2000;

        // PEDRO PATHING STUFF
        startingPose = new Pose(56, 9, Math.toRadians(90));
        automatedDrive = false;
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // because the pathChain is defined as a SUPPLIER, the operations inside are ONLY executed
        // when the get() method is explicitly called, NOT when the supplier is defined
        // :: this operator creates a compact, easy to read lambda expression for methods that already have a name
        // lambda expression can be created and executed on demand
        pathChain = () -> follower.pathBuilder() // Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(59,11))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,Math.toRadians(110), 0.8))
                .build();

        pathChainGoal = () -> follower.pathBuilder() // Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(30,120))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading,Math.toRadians(135), 0.8))
                .build();



    }

    @Override public void start() {
        follower.startTeleOpDrive(true);

    }

    @Override public void loop() {
        // Call Pedro once per loop
        follower.update();
        telemetryM.update();

        double drive = -gamepad1.left_stick_y;
        //drive *= Math.abs(drive);
        double strafe = -gamepad1.left_stick_x;
        //strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        //turn *= Math.abs(turn);

        //mecanumDrive.driveRobotCentric(drive, strafe, turn);
        if (!automatedDrive) {
            //mecanumDrive.driveFieldCentric(drive, strafe, turn);
            follower.setTeleOpDrive(drive,strafe,turn,true);
        }
        if (gamepad1.dpadUpWasPressed()) {
            follower.followPath(pathChain.get(),0.9, true);
            automatedDrive = true;
        } else if (gamepad1.dpadRightWasPressed()) {
            follower.followPath(pathChainGoal.get());
            automatedDrive = true;
        }
        if (automatedDrive && (gamepad1.dpadDownWasPressed() || !follower.isBusy())) {
            follower.startTeleOpDrive(true);
            automatedDrive = false;
        }

        if (gamepad1.options) {
            mecanumDrive.resetImu();
        }

        // GAMEPAD2 manual controls

        intake.spin(gamepad2.left_trigger - gamepad2.right_trigger);
        shooterVelocity = shooter.velocity();


        if (gamepad2.rightBumperWasPressed()) {
            shooter.spin(shooterTargetVelocity);
        } else if (gamepad2.left_bumper) {
            if (shooterVelocity > 1000) {
                shooter.spin(0.0);
            } else {
                shooter.spin(-500);
            }
        } else if (gamepad2.leftBumperWasReleased()) {
            shooter.spin(0.0);
        }

        if (gamepad2.dpad_up && (shooterVelocity>(shooterTargetVelocity-50))) {
            flipper.up();
        } else if (gamepad2.dpad_down) {
            flipper.down();
        }

        // Gamepad2 automatic scoring controls

        if (gamepad2.yWasPressed()) {
            automaticShooting = AutomaticShooting.AUTO1;
            shooter.motorvelocity = (int) shooterTargetVelocity;
            shooter.score(true,1,telemetry);
        } else if (gamepad2.aWasPressed()) {
            automaticShooting = AutomaticShooting.AUTO3;
            shooter.score(true, 3,telemetry);
        }

        if ((automaticShooting == AutomaticShooting.AUTO1) || (automaticShooting == AutomaticShooting.AUTO3)) {
            if (shooter.score(false,3,telemetry)) {
                automaticShooting = AutomaticShooting.MANUAL;
            }
        }


        aprilTagSubsystem.update();
        results = aprilTagSubsystem.angleANDdistance(20);
        distanceAprilTag = results[0];
        angleAprilTag = results[1];
        indicatorLights.display(distanceAprilTag, currentRange, angleAprilTag, 0.0); // should be actual ideal distance for shooting

        // GAMEPAD1 controls

        if (gamepad1.leftBumperWasPressed()) {
            shooterTargetVelocity = shooterTargetVelocity - 5;
        } else if (gamepad1.rightBumperWasPressed()) {
            shooterTargetVelocity = shooterTargetVelocity + 5;
        }

        if (gamepad1.yWasPressed()) {
            // near
            shooterTargetVelocity = shooter.NEARVELOCITY;
            shooter.spin(shooterTargetVelocity);
            currentRange = idealRanges[0];
            //shooter.setShooterVelocity(0);
            shooterDistance = "Near";
        } else if (gamepad1.xWasPressed()) {
            // medium
            shooterTargetVelocity = shooter.MEDIUMVELOCITY;
            shooter.spin(shooterTargetVelocity);
            currentRange = idealRanges[1];
            //shooter.setShooterVelocity(1);
            shooterDistance = "Medium";
        } else if (gamepad1.bWasPressed()) {
            // far
            //shooter.setShooterVelocity(2);
            shooterTargetVelocity = shooter.FARVELOCITY;
            shooter.spin(shooterTargetVelocity);
            currentRange = idealRanges[2];
            shooterDistance = "Far";
        } else if (gamepad1.aWasPressed()) {
            // really far
            //shooter.setShooterVelocity(3);
            shooterTargetVelocity = shooter.REALLYFARVELOCITY;
            shooter.spin(shooterTargetVelocity);
            currentRange = idealRanges[3];
            shooterDistance = "Really Far";
        }

        if (DEBUG) {
            telemetry.addData("SHOOTER DISTANCE", shooterDistance);
            telemetry.addData("SHOOTER TARGET VELOCITY",shooterTargetVelocity);
            telemetry.addData("SHOOTER ACTUAL VELOCITY",shooterVelocity);

            telemetry.addData("RANGE",distanceAprilTag);
            telemetry.addData("ANGLE",angleAprilTag);
        }

        aprilTagSubsystem.debug(telemetry,20);
        telemetry.update();
    } // end loop

    public boolean checkRange() {

        double distance;
        distance = 0;

        return false;
    }
} // end class TeleOpByTylerNov26

