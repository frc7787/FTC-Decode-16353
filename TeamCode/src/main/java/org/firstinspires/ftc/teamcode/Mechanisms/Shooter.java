package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;


public class Shooter {

    private final DcMotorEx motor;
    private final DcMotorEx motor2;

    private Flipper flipper;
    private Intake intake;
    private Timer scoreTimer;
    private  boolean started = false;
    private Timer shooterTimer;

    private int MOTORVELOCITY = 2000;
    private enum shootingState{
        START,INTAKE,MOTORSPINUP,FLINGER,END
    }
    private shootingState shooterState = shootingState.START;

    public Shooter(HardwareMap hardwareMap) {

        flipper = new Flipper(hardwareMap);
        intake = new Intake(hardwareMap);
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motor2 = hardwareMap.get(DcMotorEx.class, "shooterTwo");
        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        scoreTimer = new Timer();
        shooterTimer = new Timer();
    }



    public void spin(double velocity) {

        motor.setVelocity(velocity);
        motor2.setVelocity(velocity);
    }

    public double velocity() {
        return motor2.getVelocity();
    }

    public void update() {
        switch (shooterState) {
            case START:
                if(gamepad2.cross){
                    motor2.setVelocity(MOTORVELOCITY);
                    motor.setVelocity(MOTORVELOCITY);

                    shooterTimer.resetTimer();
                    shooterState = shootingState.INTAKE;
                }

            case INTAKE:
                intake.spin(0.5);
                if (shooterTimer.getElapsedTimeSeconds() > 3) {
                    shooterState = shootingState.MOTORSPINUP;


                }
            case MOTORSPINUP:

                if(motor.getVelocity()>MOTORVELOCITY- 100 &&motor2.getVelocity()>MOTORVELOCITY- 100) {
                    shooterTimer.resetTimer();
                    shooterState = shootingState.FLINGER;

                }

            case FLINGER:

                flipper.up();
                if(shooterTimer.getElapsedTimeSeconds()>2){
                    flipper.down();

                } else if (shooterTimer.getElapsedTimeSeconds()>4) {
                    shooterState = shootingState.END;
                }
            case END:
                break;
        }
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
