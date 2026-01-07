package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MechanismPortTest")
@Disabled
public class MechanismPortTest extends OpMode {
    private AprilTagSubsystem aprilTagSubsystem;
    private Intake intake;
    private Shooter shooter;

    private DcMotorEx chPort1, chPort2, chPort3, chPort0, ehPort1, ehPort2, ehPort3, ehPort0;

    private Servo servo;

    private double servoPosition = 0;

    @Override
    public void init() {

        chPort1 = hardwareMap.get(DcMotorEx.class, "chport1");
        chPort2 = hardwareMap.get(DcMotorEx.class, "chport2");
        chPort3 = hardwareMap.get(DcMotorEx.class, "chport3");
        chPort0 = hardwareMap.get(DcMotorEx.class, "chport0");
        ehPort1 = hardwareMap.get(DcMotorEx.class, "ehport1");
        ehPort2 = hardwareMap.get(DcMotorEx.class, "ehport2");
        ehPort3 = hardwareMap.get(DcMotorEx.class, "ehport3");
        ehPort0 = hardwareMap.get(DcMotorEx.class, "ehport0");

        servo = hardwareMap.get(Servo.class, "servo");

        servo.setPosition(0);



    }

    @Override
    public void init_loop() {


    }

    @Override
    public void loop() {

        if (gamepad1.rightBumperWasPressed()) {
            servoPosition = servoPosition + 0.05;
        } else if (gamepad1.leftBumperWasPressed()) {
            servoPosition = servoPosition - 0.05;
        }
        servo.setPosition(servoPosition);

        telemetry.addData("servoPosition", servoPosition);
        telemetry.update();

    }

    @Override
    public void start() {



    }

    @Override
    public void stop() {

    }
}
