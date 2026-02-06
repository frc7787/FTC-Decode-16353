package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
public class Gate {
    private final Servo gateServo;

    public static double CLOSED_POSITION = 0.5;
    public static double OPEN_POSITION   = 0.79;

    public Gate(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setPosition(CLOSED_POSITION);
    }

    public void closed() {
        gateServo.setPosition(CLOSED_POSITION);
    }

    public void open() {
        gateServo.setPosition(OPEN_POSITION);
    }

}
