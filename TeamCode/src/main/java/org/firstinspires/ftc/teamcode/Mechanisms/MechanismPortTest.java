package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MechanismPortTest extends OpMode {
    private AprilTagSubsystem aprilTagSubsystem;
    private Intake intake;
    private Shooter shooter;

    private DcMotorEx chPort1, chPort2, chPort3, chPort0, ehPort1, ehPort2, ehPort3, ehPort0;

    @Override
    public void init() {

        chPort1 = hardwareMap.get(DcMotorEx.class, "port1");
        chPort2 = hardwareMap.get(DcMotorEx.class, "port2");
        chPort3 = hardwareMap.get(DcMotorEx.class, "port3");
        chPort0 = hardwareMap.get(DcMotorEx.class, "port0");
        ehPort1 = hardwareMap.get(DcMotorEx.class, "port1");
        ehPort2 = hardwareMap.get(DcMotorEx.class, "port2");
        ehPort3 = hardwareMap.get(DcMotorEx.class, "port3");
        ehPort0 = hardwareMap.get(DcMotorEx.class, "port0");

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {

    }

    @Override
    public void start() {

    }

    @Override
    public void stop() {

    }
}
