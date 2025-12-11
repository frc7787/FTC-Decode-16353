package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.Flipper;
import org.firstinspires.ftc.teamcode.Mechanisms.IndicatorLights;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class  teleOpByGhyth extends OpMode {
    private DcMotor backRightDrive, frontRightDrive, backLeftDrive, frontLeftDrive;

    private double shooterTargetVelocity;
    private boolean scoringThree = false;
    private boolean scoringOne = false;


    private boolean DEBUG = true;

    private double[] results;

    private double shooterVelocity = 0;

    private boolean lastOptions = false;
    private AprilTagSubsystem aprilTagSubsystem;
    private String shooterDistance;

    private double distanceAprilTag = 9999;
    private double angleAprilTag = 9999;

    private MecanumDriveBase mecanumDrive;
    private Intake intake;
    private Shooter shooter;
    private boolean AUTOSCORE = false;
    private Flipper flipper;
     private IndicatorLights indicatorLights;


    private IMU imu;

    public void init() {

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        flipper = new Flipper(hardwareMap);
        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);
        mecanumDrive = new MecanumDriveBase(hardwareMap);




        // drive motor init





        telemetry.addLine("Simple Field-Centric Initialized");
        telemetry.update();





    }

    @Override
    public void loop() {
        //Drive code

        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);

        mecanumDrive.driveFieldCentric(drive, strafe, turn);
        
       // Robot heading

        intake.spin(gamepad2.left_trigger - gamepad2.right_trigger) ;
        shooterVelocity = shooter.velocity();
        if (gamepad1.options) {
            mecanumDrive.resetImu();
        }
        if (gamepad1.triangleWasPressed()) {
            // near
            shooter.setShooterVelocity(0);
            shooter.spin(shooter.NEARVELOCITY);
            shooterDistance = "Near";
        } else if (gamepad1.squareWasPressed()) {
            shooter.spin(shooter.MEDIUMVELOCITY);
            // medi
            shooter.setShooterVelocity(1);

            shooterDistance = "Medium";
        } else if (gamepad1.circleWasPressed()) {
            // far
            shooter.setShooterVelocity(2);

            shooter.spin(shooter.FARVELOCITY);
            shooterDistance = "Far";
        } else if (gamepad1.crossWasPressed()) {
            // really fa
            shooter.spin(shooter.REALLYFARVELOCITY);
            shooter.setShooterVelocity(3);

            shooterDistance = "Really Far";
        }

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
        if (gamepad1.leftBumperWasPressed()) {
            shooterTargetVelocity = shooterTargetVelocity - 5;
        } else if (gamepad1.rightBumperWasPressed()) {
            shooterTargetVelocity = shooterTargetVelocity + 5;
        }





        if (gamepad2.a && !scoringThree) {
            scoringThree = true;
            shooter.startScoring = true;    // reset shooter
        }

        if (scoringThree) {
            if (shooter.score(true, 3, telemetry)) {
                scoringThree = false;       // finished scoring all 3 balls
            }
        }

        if (gamepad2.x && !scoringOne) {
            scoringOne = true;
            shooter.startScoring = true;
        }

        if (scoringOne) {
            if (shooter.score(true, 1, telemetry)) {
                scoringOne = false;         // finished firing 1 ball
            }
        }
        if (DEBUG) {
            telemetry.addData("SHOOTER DISTANCE", shooterDistance);
            telemetry.addData("SHOOTER TARGET VELOCITY",shooterTargetVelocity);
            telemetry.addData("SHOOTER ACTUAL VELOCITY",shooterVelocity);

            telemetry.addData("RANGE",distanceAprilTag);
            telemetry.addData("ANGLE",angleAprilTag);
        }









    }
}



