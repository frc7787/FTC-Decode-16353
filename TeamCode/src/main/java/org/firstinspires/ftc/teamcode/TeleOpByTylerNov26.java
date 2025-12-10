package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.Flipper;
import org.firstinspires.ftc.teamcode.Mechanisms.IndicatorLights;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDriveBase;

@TeleOp(name = "TeleOpByTyler", group = "$")
public class TeleOpByTylerNov26 extends OpMode {

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

    @Override public void init() {
        mecanumDrive = new MecanumDriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        flipper = new Flipper(hardwareMap);

        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);
        indicatorLights = new IndicatorLights(hardwareMap);

        shooterTargetVelocity = 2000;
    }

    @Override public void start() {

    }

    @Override public void loop() {
        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);

        mecanumDrive.driveRobotCentric(drive, strafe, turn);

        if (gamepad1.options) {
            mecanumDrive.resetImu();
        }

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

        if (gamepad2.dpad_up) {
            flipper.up();
        } else if (gamepad2.dpad_down) {
            flipper.down();
        }



        aprilTagSubsystem.update();
        results = aprilTagSubsystem.angleANDdistance(20);
        distanceAprilTag = results[0];
        angleAprilTag = results[1];
        indicatorLights.display(distanceAprilTag,150.0, angleAprilTag, 0.0); // should be actual ideal distance for shooting

        if (gamepad1.leftBumperWasPressed()) {
            shooterTargetVelocity = shooterTargetVelocity - 5;
        } else if (gamepad1.rightBumperWasPressed()) {
            shooterTargetVelocity = shooterTargetVelocity + 5;
        }

        if (gamepad1.triangleWasPressed()) {
            // near
            shooter.setShooterVelocity(0);
            shooterDistance = "Near";
        } else if (gamepad1.squareWasPressed()) {
            // medium
            shooter.setShooterVelocity(1);
            shooterDistance = "Medium";
        } else if (gamepad1.circleWasPressed()) {
            // far
            shooter.setShooterVelocity(2);
            shooterDistance = "Far";
        } else if (gamepad1.xWasPressed()) {
            // really far
            shooter.setShooterVelocity(3);
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

