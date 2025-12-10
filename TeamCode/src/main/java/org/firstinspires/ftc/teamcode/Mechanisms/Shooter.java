package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Shooter {

    private final DcMotorEx motor;
    private final DcMotorEx motor2;

    private Flipper flipper;
    private Intake intake;
    private Timer scoreTimer;
    private  boolean started = false;
    private Timer shooterTimer;

    public int motorvelocity = 2000;
    public int NEARVELOCITY = 1710;
    public int MEDIUMVELOCITY = 1800;
    public int FARVELOCITY = 2015;
    public int REALLYFARVELOCITY = 2110;
    private enum shootingState{
        IDLE, START,INTAKE,MOTORSPINUP,FLINGER,END
    }
    private shootingState shooterState;
    private boolean startScoring = true;
    private double totalBalls = 3;

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

        shooterState = shootingState.IDLE;
    }

    public void setShooterVelocity(int velocity) {
        if (velocity == 0) {
            motorvelocity= NEARVELOCITY;
        } else if (velocity == 1) {
            motorvelocity = MEDIUMVELOCITY;
        } else if (velocity == 2) {
            motorvelocity = FARVELOCITY;
        } else if (velocity == 3) {
            motorvelocity = REALLYFARVELOCITY;
        }
    }



    public void spin(double velocity) {

        motor.setVelocity(velocity);
        motor2.setVelocity(velocity);
    }

    public double velocity() {
        return motor2.getVelocity();
    }

    public boolean update(boolean startShootingProcess, boolean cancelShootingProcess, Telemetry telemetry) {
        if (cancelShootingProcess) {
            intake.spin(0);
            flipper.down();
            startScoring =true;
            shooterState = shootingState.IDLE;
            return false;
        } else {
            switch (shooterState) {
                case IDLE: {
                    if (startShootingProcess) {
                        shooterState = shootingState.START;
                    }
                    telemetry.addData("SHOOTER UPDATE","IDLE");
                    break;
                }
                case START: {
                    motor2.setVelocity(motorvelocity);
                    motor.setVelocity(motorvelocity);

                    shooterTimer.resetTimer();
                    shooterState = shootingState.INTAKE;
                    intake.spin(1.0);
                    telemetry.addData("SHOOTER UPDATE","START");
                    break;
                }
                case INTAKE: {
                    intake.spin(1.0);
                    if (shooterTimer.getElapsedTimeSeconds() > 0.6) {
                        shooterState = shootingState.MOTORSPINUP;
                    }
                    telemetry.addData("SHOOTER UPDATE","INTAKE");
                    break;
                }
                case MOTORSPINUP: {
                    if (motor.getVelocity() > motorvelocity - 50) {
                        shooterTimer.resetTimer();
                        shooterState = shootingState.FLINGER;
                        flipper.up();
                    }
                    telemetry.addData("SHOOTER UPDATE","MOTORSPINUP");
                    break;
                }
                case FLINGER: {
                    if (shooterTimer.getElapsedTimeSeconds() > 0.8) { // was 1.5
                        shooterState = shootingState.END;
                    } else if (shooterTimer.getElapsedTimeSeconds() > 0.3) {  // was 1
                        flipper.down();
                    }
                    telemetry.addData("SHOOTER UPDATE","FLINGER");
                    break;
                }
                case END: {
                    telemetry.addData("SHOOTER UPDATE","END");
                    shooterState = shootingState.IDLE;
                    return true; // update returns true only when the cycle has finally reached "END"
                }
            } // end switch
            return false; // update returns false if the cycle has not JUST finished "scoring"
        } // end else, for switch

    } // end update

    public boolean score(boolean placeHolder, double numberBalls, Telemetry telemetry) {

        if (startScoring) {
            telemetry.addData("SHOOTER SCORE","startScoring");
            totalBalls = numberBalls;
            startScoring = false;
            this.update(true,false, telemetry); // start the shooting update process, with "true" for shootingstate START
        } else if (this.update(false,false, telemetry)) {
            telemetry.addData("SHOOTER SCORE", "update true, so minus one ball");
            totalBalls = totalBalls - 1;
            if (totalBalls == 0) {
                telemetry.addData("SHOOTER SCORE", "total balls equals ZERO");
                startScoring = true;
                return true; // finished firing all balls, return true for "score"
            } else {
                this.update(true, false, telemetry); // start the shooting update process, with "true" for shootingstate START
                return false; // if not finished firing all balls, return false for "score"
            }
        }
        return false;
    } // end score
}
