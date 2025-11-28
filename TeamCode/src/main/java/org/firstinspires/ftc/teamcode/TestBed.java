package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name="TestBed")
public class TestBed extends OpMode {

    // global variable declarations
    private DcMotorEx leftDrive = null;

    private Servo myServo = null;

    private float leftDrivePower = 0;

    @Override
    public void init() {

        leftDrive = hardwareMap.get(DcMotorEx.class, "left_drive");

        myServo = hardwareMap.get(Servo.class, "myServo");

        leftDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftDrive.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

    } // end init

    @Override
    public void init_loop() {

    } // end init_loop

    @Override
    public void start() {

    } // end start

    @Override
    public void loop() {

        if (gamepad1.left_stick_y!=0) { // using power from the left stick
            leftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            leftDrivePower = - gamepad1.left_stick_y;
            leftDrive.setPower(leftDrivePower);
        } else if (gamepad1.aWasPressed()) { // set velocity
            leftDrive.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            leftDrive.setVelocity(200);
        } else if (gamepad1.bWasPressed()) { // set target position - note the SEQUENCE of commands
            leftDrive.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            leftDrive.setTargetPosition(2000);
            leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftDrive.setPower(0.5);
            while (leftDrive.isBusy()) {
                leftDrive.setPower(0.5);
            }
            leftDrive.setPower(0);
        } else if (gamepad1.circleWasPressed()) {
            leftDrive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            leftDrive.setPower(0);
        }


    } // end loop
}
