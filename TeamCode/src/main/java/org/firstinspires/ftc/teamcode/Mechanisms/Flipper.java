package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Flipper {
    private final Servo servoFlipper;

    public Flipper(HardwareMap hardwareMap) {
        servoFlipper = hardwareMap.get(Servo.class, "servoFlipper");
        servoFlipper.setPosition(0.05);
    }

    public void up() {
        servoFlipper.setPosition(0.35);
    }

    public void down() {
        servoFlipper.setPosition(0.05);
    }

}
