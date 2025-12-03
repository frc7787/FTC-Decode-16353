package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Shooter {

    private final DcMotorEx motor;

    private Timer scoreTimer;

    private  boolean started = false;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        scoreTimer = new Timer();
    }

    public void spin(double power) {
        motor.setPower(power);
    }

    public boolean score(double numberBalls) {
        if (!started) {
            scoreTimer.resetTimer();
            started = true;
            this.spin(0.8);
        } else if (scoreTimer.getElapsedTimeSeconds() > 2* numberBalls) {
            started = false;
            this.spin(0);
            return true; // finished scoring, so score is true
        }
        return false;
    } // end score
}
