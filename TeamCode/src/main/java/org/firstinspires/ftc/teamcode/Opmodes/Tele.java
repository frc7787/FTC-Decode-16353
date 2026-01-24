package org.firstinspires.ftc.teamcode.Opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.Flipper;
import org.firstinspires.ftc.teamcode.Mechanisms.IndicatorLights;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.TeleOpByTylerNov26;

@Disabled
@TeleOp(name = "Tele", group = "$")
public class Tele extends OpMode {

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

        //mecanumDrive.driveRobotCentric(drive, strafe, turn);
        mecanumDrive.driveFieldCentric(drive,strafe,turn);

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
        indicatorLights.display(IndicatorLights.targeting.GREEN, angleAprilTag, 0.0); // should be actual ideal distance for shooting

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
}
