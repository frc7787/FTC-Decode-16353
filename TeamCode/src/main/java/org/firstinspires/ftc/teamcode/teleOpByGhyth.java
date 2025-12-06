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

    private boolean lastOptions = false;
    private AprilTagSubsystem aprilTagSubsystem;

    private MecanumDriveBase mecanumDrive;
    private Intake intake;
    private Shooter shooter;

    private Flipper flipper;
    private boolean DEBUG = true;


    private IMU imu;

    public void init() {

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        flipper = new Flipper(hardwareMap);
        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);



        // drive motor init
        backRightDrive = hardwareMap.get(DcMotor.class, "backRightDrive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "backLeftDrive");
        frontLeftDrive = hardwareMap.get(DcMotor.class, "frontLeftDrive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRightDrive");

        frontLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        frontLeftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightDrive.setDirection(DcMotorEx.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotorEx.Direction.REVERSE);


        imu = hardwareMap.get(IMU.class, "imu");
        final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(IMU_PARAMETERS);



        telemetry.addLine("Simple Field-Centric Initialized");
        telemetry.update();





    }

    @Override
    public void loop() {
        //Drive code
        double drive = -gamepad1.left_stick_y; // forward/backwards
        double strafe = gamepad1.left_stick_x; // left/right
        double turn = gamepad1.right_stick_x; // rotation
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Robot heading
        double adjustedStrafe = strafe * Math.cos(heading) + drive * Math.sin(heading);
        double adjustedDrive = strafe * Math.sin(heading) + drive * Math.cos(heading);

        double fl = adjustedDrive + adjustedStrafe + turn;
        double fr = adjustedDrive - adjustedStrafe - turn;
        double bl = adjustedDrive - adjustedStrafe + turn;
        double br = adjustedDrive + adjustedStrafe - turn;

        // Normalize
        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        // Set powers using your motor names
        frontLeftDrive.setPower(fl / max);
        frontRightDrive.setPower(fr / max);
        backLeftDrive.setPower(bl / max);
        backRightDrive.setPower(br / max);

        // Reset IMU with OPTIONS button
        boolean optionsPressed = gamepad1.options && !lastOptions;
        if (optionsPressed) imu.resetYaw();
        lastOptions = gamepad1.options;

        telemetry.addData("Heading (deg)", Math.toDegrees(heading));





        // end of drive code
        if (gamepad1.options) {
            mecanumDrive.resetImu();
        }

        intake.spin(gamepad2.left_trigger - gamepad2.right_trigger);

        if (gamepad2.rightBumperWasPressed()) {
            shooter.spin(2000);
        } else if (gamepad2.left_bumper) {
            if (shooter.velocity() > 1000) {
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

        if (DEBUG) {
            telemetry.addData("SHOOTER VELOCITY",shooter.velocity());
            telemetry.update();
        }

        aprilTagSubsystem.debug(telemetry,20);


    }
}



