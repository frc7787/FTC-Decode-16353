package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Shooter {

    private final DcMotorEx motor;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
    }

    public void spin(double power) {
        motor.setPower(power);
    }
}
