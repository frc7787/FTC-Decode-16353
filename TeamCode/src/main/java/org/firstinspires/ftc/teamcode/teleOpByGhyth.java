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
    private String shooterDistance;

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
    
        // Normalize
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
        } else if (gamepad1.crossWasPressed()) {
            // really far
            shooter.setShooterVelocity(3);
            shooterDistance = "Really Far";
        }
        boolean startShoot = gamepad2.right_bumper;

        boolean cancelShoot = gamepad2.crossWasPressed();


        boolean finishedOneShot = shooter.update(startShoot, cancelShoot, telemetry);

        if (finishedOneShot) {
            telemetry.addLine("Shooter Cycle Complete!");
        }

        // ---------------- AUTO MULTI-SHOT ----------------
        if (gamepad2.left_bumper) {
            shooter.score(true, 3, telemetry);   // shoot 3 rings automatically
        }



        // end of drive code
        if (gamepad1.options) {
            mecanumDrive.resetImu();
        }
        

        

    }
}



