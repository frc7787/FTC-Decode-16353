package org.firstinspires.ftc.teamcode.Mechanisms;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Shooter {

    private final DcMotorEx motor;

    private Timer scoreTimer;

    private  boolean started = false;

    public Shooter(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scoreTimer = new Timer();
    }

    public void spin(double velocity) {
        motor.setVelocity(velocity);
    }

    public double velocity() {
        return motor.getVelocity();
    }


    public boolean score(double numberBalls) {
        if (!started) {
            scoreTimer.resetTimer();
            started = true;
            this.spin(2000);
        } else if (scoreTimer.getElapsedTimeSeconds() > 2* numberBalls) {
            started = false;
            this.spin(0);
            return true; // finished scoring, so score is true
        }
        return false;
    } // end score
}
