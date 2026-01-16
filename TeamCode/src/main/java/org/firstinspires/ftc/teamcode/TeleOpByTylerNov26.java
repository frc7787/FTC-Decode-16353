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


    @Override public void init() {
        mecanumDrive = new MecanumDriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        flipper = new Flipper(hardwareMap);

        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);
        indicatorLights = new IndicatorLights(hardwareMap);

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

        //Changed here

        if (gamepad1.triangle) shooter.setShooterVelocity(0); //near velocity
        if (gamepad1.square) shooter.setShooterVelocity(1); //medium velocity
        if (gamepad1.circle) shooter.setShooterVelocity(2); //far velocity
        if (gamepad1.x) shooter.setShooterVelocity(3); //really far velocity

        boolean startSingleShot = gamepad2.rightBumperWasPressed(); //Shoot one ball
        boolean cancelShot = gamepad2.leftBumperWasPressed(); //Cancel Shooting
        boolean startThreeShot = gamepad2.yWasPressed(); //Shoot three balls

        shooter.score(startSingleShot, 1, telemetry); //single shot
        shooter.score(startThreeShot, 3, telemetry); //multi shot
        telemetry.addData("Shooter Velocity", shooter.velocity()); // shows current speed


        if (DEBUG) {
            telemetry.addData("RANGE",distanceAprilTag);
            telemetry.addData("ANGLE",angleAprilTag);
            telemetry.addData("SHOOTER VELOCITY", shooter.velocity());
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

