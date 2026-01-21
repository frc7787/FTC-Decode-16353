package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.function.Supplier;

@TeleOp(name = "TeleJanuary", group = "$")
public class TeleJanuary extends OpMode {

    // PEDRO PATHING STUFF
    private Follower follower;
    public static Pose startingPose;
    private boolean automatedDrive;

    private boolean automatedTargeting;
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
    AprilTagDetection currentTagValues;

    private boolean currentTagDetected;
    private double distanceAprilTag = 9999;
    private double angleAprilTag = 9999;
    private double shooterVelocity = 0;
    private double shooterTargetVelocity;
    private String shooterDistance = "Default";
    private enum AutomaticShooting {MANUAL, AUTO1,AUTO3};
    private AutomaticShooting automaticShooting = AutomaticShooting.MANUAL;

    private double[] idealRanges = {107,172,260,295};
    private double[] shootingAngles = {0,5,10,10,10,10,5,5,5,5,-5,0,5,10}; // 13 indicated zone by number
    private double[] targetVelocity = {0,1,2,3,4,5,6,7,8,9,10,11,12,13}; // 13 indicated zone by number
    private double currentRange = 170;

    @Override public void init() {
        mecanumDrive = new MecanumDriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        flipper = new Flipper(hardwareMap);

        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);
        indicatorLights = new IndicatorLights(hardwareMap);

        shooterTargetVelocity = 2000;
        automatedTargeting = true;

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

        // PEDRO DRIVING !!!!
        // this needs to be enabled for automatic pedro driving, but is probably the cause of the
        // difficulties when using mecanumDrive.driveFieldCentric
        //follower.startTeleOpDrive(true);
        aprilTagSubsystem.setManualExposure(telemetry, 10, 100, 2500);
    }

    @Override public void loop() {

        // Call Pedro once per loop
        follower.update();
        telemetryM.update();

        // DRIVE CONTROLS

        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);

        mecanumDrive.driveFieldCentric(drive, strafe, turn);

        if (gamepad1.options) {
            mecanumDrive.resetImu();
        }


        intake.spin(gamepad2.left_trigger - gamepad2.right_trigger);
        shooterVelocity = shooter.velocity();

        // GAMEPAD 1 - MANUAL control of the SHOOTER speed

        if (gamepad1.yWasPressed()) {
            // near
            shooterTargetVelocity = shooter.NEARVELOCITY;
            shooter.spin(shooterTargetVelocity);
            //shooter.setShooterVelocity(0);
            shooterDistance = "Near";
            automatedTargeting = false;
        } else if (gamepad1.xWasPressed()) {
            // medium
            shooterTargetVelocity = shooter.MEDIUMVELOCITY;
            shooter.spin(shooterTargetVelocity);
            //shooter.setShooterVelocity(1);
            shooterDistance = "Medium";
            automatedTargeting = false;
        } else if (gamepad1.bWasPressed()) {
            // far
            //shooter.setShooterVelocity(2);
            shooterTargetVelocity = shooter.FARVELOCITY;
            shooter.spin(shooterTargetVelocity);
            shooterDistance = "Far";
            automatedTargeting = false;
        } else if (gamepad1.aWasPressed()) {
            // really far
            //shooter.setShooterVelocity(3);
            shooterTargetVelocity = shooter.REALLYFARVELOCITY;
            shooter.spin(shooterTargetVelocity);
            shooterDistance = "Really Far";
            automatedTargeting = false;
        } else if (gamepad1.shareWasPressed()) {
            automatedTargeting = !automatedTargeting; // toggle automatedTargeting ON/OFF
        }   // turn ON automatedTargeting

        // GAMEPAD1 - fine control of shooter velocity

        if (gamepad1.leftBumperWasPressed()) {
            shooterTargetVelocity = shooterTargetVelocity - 10;
            shooter.spin(shooterTargetVelocity);
        } else if (gamepad1.rightBumperWasPressed()) {
            shooterTargetVelocity = shooterTargetVelocity + 10;
            shooter.spin(shooterTargetVelocity);
        }

        // GAMEPAD2 manual controls for turning shooter ON and OFF

        if (gamepad2.rightBumperWasPressed()) {
            automatedTargeting = true;
            shooter.spin(shooterTargetVelocity);
        } else if (gamepad2.left_bumper) {
            automatedTargeting = false;
            if (shooterVelocity > 1000) {
                shooterTargetVelocity = 0;
                shooter.spin(0.0);
            } else {
                shooterTargetVelocity = 0;
                shooter.spin(-500);
            }
        } else if (gamepad2.leftBumperWasReleased()) {
            shooter.spin(0.0);
        }

        // GAMEPAD 2 - MANUAL control of flipper

        //if (gamepad2.dpad_up && (shooterVelocity>(shooterTargetVelocity-75))) {
        if (gamepad2.dpad_up) {
            intake.spin(1.0);
            flipper.up();
        } else if (gamepad2.dpad_down) {
            flipper.down();
        }

        // PEDRO LOCALIZATION - not currently used except for ZONE calculation

        telemetry.addLine(String.format("Current Pedro Pose in loop x %6.1f y %6.1f heading rad %6.1f",
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()));
        aprilTagSubsystem.update();
        //aprilTagSubsystem.debug(telemetry,20);

        if (!automatedDrive) {
            currentTagDetected = localizationUpdate();
            if (automatedTargeting && currentTagDetected) {
                targetingUpdate(); // update shooterTargetVelocity
            } else if (currentTagDetected){ // robot is detecting April Tag but autotargeting is OFF
                indicatorLights.display(IndicatorLights.targeting.RED, 99.9, 0.0); // LED RED for April Tag, but autotargeting OFF
            } else { // robot is not detecting a tag or autotargeting is off
                indicatorLights.display(IndicatorLights.targeting.OFF, 99.9, 0.0); // LED OFF for no April Tag, no autotargeting
            }
        }

        if (DEBUG) {
            telemetry.addData("SHOOTER DISTANCE", shooterDistance);
            telemetry.addData("SHOOTER TARGET VELOCITY",shooterTargetVelocity);
            telemetry.addData("SHOOTER ACTUAL VELOCITY",shooterVelocity);

            telemetry.addData("RANGE",distanceAprilTag);
            telemetry.addData("ANGLE",angleAprilTag);
        }

        //aprilTagSubsystem.debug(telemetry,20);
        telemetry.update();
    } // end loop

    public boolean localizationUpdate() {
        // at this point we want to check if we have a valid GOAL April Tag available
        // if YES, then we update the localizer based on the Kalman Filtered April Tag robot pose
        // keeping in mind that in the AprilTagDetection object, we are storing the robot pose in the FTCPose
        AprilTagDetection tag;
        Pose cameraPose;
        double x, y, heading;
        double idealShootingAngle;

        if (aprilTagSubsystem.tagDetectedWithId(20)) {
            currentTagValues = aprilTagSubsystem.tagWithId(20);
            // convert from FTC Decode RobotPose to PedroPathing
            // using the audience side perspective
            // FTC Decode RobotPose: positive y axis right, negative x axis forward (orients better from blue goal side)
            // PedroPathing: positive x axis right, positive y axis forward
            x = currentTagValues.ftcPose.y + 72;
            y = -currentTagValues.ftcPose.x + 72;
            heading = follower.getHeading();
            telemetry.addLine(String.format("Current Pedro Pose %6.1f %6.1f %6.1f (inch)",
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getPose().getHeading()));
            cameraPose = new Pose(-x, -y, heading, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            follower.setPose(cameraPose);
            telemetry.addLine(String.format("Pedro Pose updated from April Tag %6.1f %6.1f %6.1f (inch)",
                    x,
                    y,
                    heading));
            return true;

        } // tag 20 visible BLUE Goal
        // TAG 24 RED goal
        else if (aprilTagSubsystem.tagDetectedWithId(24)) {
            currentTagValues = aprilTagSubsystem.tagWithId(24);
            // convert from FTC Decode RobotPose to PedroPathing
            // using the audience side perspective
            // FTC Decode RobotPose: positive y axis right, negative x axis forward (orients better from blue goal side)
            // PedroPathing: positive x axis right, positive y axis forward
            x = currentTagValues.ftcPose.y + 72;
            y = -currentTagValues.ftcPose.x + 72;
            heading = follower.getHeading();
            telemetry.addLine(String.format("Current Pedro Pose %6.1f %6.1f %6.1f (inch)",
                    follower.getPose().getX(),
                    follower.getPose().getY(),
                    follower.getPose().getHeading()));
            cameraPose = new Pose(-x, -y, heading, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            follower.setPose(cameraPose);
            telemetry.addLine(String.format("Pedro Pose updated from April Tag %6.1f %6.1f %6.1f (inch)",
                    x,
                    y,
                    heading));
            return true;

        } // end of tag 24 visible RED Goal
        else {return false;}
    } // end localizationUpdate

    public void targetingUpdate() {
        // at this point we want to check if we have a valid GOAL April Tag available
        // if YES, then we update the localizer based on the Kalman Filtered April Tag robot pose
        // keeping in mind that in the AprilTagDetection object, we are storing the robot pose in the FTCPose

        double x, y, heading;
        double idealShootingAngle;
        int zone=0;
        // convert from FTC Decode RobotPose to PedroPathing
        // using the audience side perspective
        // FTC Decode RobotPose: positive y axis right, negative x axis forward (orients better from blue goal side)
        // PedroPathing: positive x axis right, positive y axis forward
        // THEORETICALLY, the current tag FTC pose should be the correct Pedro Robot Pose, since we have just run localizationupdate()
        // x = tag.ftcPose.y + 72;
        // y = -tag.ftcPose.x + 72;
        x = currentTagValues.ftcPose.x;
        y = currentTagValues.ftcPose.y;

        distanceAprilTag = currentTagValues.ftcPose.range;
        angleAprilTag = currentTagValues.ftcPose.bearing;

        zone = calculateShootingZone(x, y);
        idealShootingAngle = shootingAngles[zone];
        shooterTargetVelocity = shooter.calculateShooterVelocity(distanceAprilTag);

        // WE have a potentially new shooter target velocity, update if the flywheel is supposed to be spinning
        // let's use automatedTargeting as a proxy for "supposed to be spinning"
        if (automatedTargeting) {
            shooter.spin(shooterTargetVelocity);
        }

        telemetry.addLine(String.format("Current Targeting: zone, range, angle, ideal angle, RPM %d %6.1f %6.1f %6.1f %6.1f",
                zone,
                distanceAprilTag,
                angleAprilTag,
                idealShootingAngle,
                shooterTargetVelocity));

        shooterDistance = String.valueOf(zone);

        indicatorLights.display(IndicatorLights.targeting.GREEN, angleAprilTag, idealShootingAngle); // should be actual ideal distance for shooting

    } // end targetingUpdate

    public int calculateShootingZone(double x, double y) {

        int zone;

        if (y<48) {
            zone = 1;
        } else {
            if (y>120) { // TOP row
                if (x>96) { // zone 5
                    zone = 5;
                } else if (x>72) { // zone 4
                    zone = 4;
                } else if (x>48) { // zone 3
                    zone = 3;
                } else { // zone 2
                    zone = 2;
                } // end of y > 120 top row

            } else if (y>96) { // second from TOP row
                if (x>96) { // zone 9
                    zone = 9;
                } else if (x>72) { // zone 8
                    zone = 8;
                } else if (x>48) { // zone 7
                    zone = 7;
                } else { // zone 6
                    zone = 6;
                } // end of y > 196 second from top row

            } else { // bottom of big launch zone
                if (x>96) { // zone 13
                    zone = 13;
                } else if (x>72) { // zone 12
                    zone = 12;
                } else if (x>48) { // zone 11
                    zone = 11;
                } else { // zone 10
                    zone = 10;
                } // end of y > 72 bottom row
            }
        }

        return zone;
    }
} // end class TeleOpByTylerNov26


