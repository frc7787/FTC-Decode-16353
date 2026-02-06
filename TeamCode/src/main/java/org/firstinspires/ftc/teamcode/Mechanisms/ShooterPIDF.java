package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ShooterPIDF {

    // FLYWHEEL configurable PIDF variables

    public static double kP = 0.0003;
    public static double kI = 0.0000;
    public static double kD = 0.00002;
    public static double kF = 0.00012;

    public static double targetRPM = 2200;

    public static double nominalVoltage = 12.0;

    // Shot readiness tuning
    public static double readyRPMTolerance = 50;   // ± RPM
    public static double readyTime = 0.15;         // seconds

    PIDFController pidf;
    VoltageSensor batteryVoltageSensor;

    private final DcMotorEx motor;
    private final DcMotorEx motor2;

    private HardwareMap hardwareMap;

    private double motorVoltage;

    public static double PIDF_F = 11.19;
    public static double PIDF_P = 60.0;

    PIDFCoefficients pidfCoefficients = new PIDFCoefficients(PIDF_P, 0, 0, PIDF_F);

    private Flipper flipper;
    private Intake intake;
    private Timer scoreTimer;
    private  boolean started = false;
    private Timer shooterTimer;

    public double motorvelocity = 2000;
    public double normalizedMotorVelocity;
    public double NEARVELOCITY = 1710;
    public double MEDIUMVELOCITY = 1800;
    public double FARVELOCITY = 2015;
    public double REALLYFARVELOCITY = 2110;

    // THESE VARIABLES ARE FOR THE AUTOMATIC APRIL TAG TARGETING range and flywheel RPM

    public static double RMP_130 = 2440;
    public static double RPM_126 = 2400;
    public static double RPM_123 = 2380;
    public static double RPM_112 = 2300;
    public static double RPM_106 = 2220;
    public static double RPM_76 = 2000;
    public static double RPM_63 = 1940;
    public static double RPM_59 = 1900;
    public static double RPM_50 = 1900;
    public static double RPM_AUDIENCE = 2290; // 2190 @12.9V; was 2290/2240 new wheel
    public static double RPM_GOAL = 1910; // was 1940 old wheel




    // THESE VARIABLES ARE FOR THE AUTOMATIC SHOOTER PROCESS.
    // They can be accessed and changed in Panels: 192.168.43.1:8001

    public static double INTAKE_TIME_START  = 0.5; // was 1.0
    public static double INTAKE_TIME_CONTINUE = 0.25; // was 0.5
    public static double VELOCITY_UPPER_OFFSET = 40;
    public static double VELOCITY_LOWER_OFFSET = 15;
    public static double FLIPPER_UP = 0.5; // was 0.5
    public static double FLIPPER_DOWN = 0.8; // was 0.8

    public static double JUST_SHOOT_IT = 1.6; // waiting for motorspinup, but at some point just shoot!


    public double[] TARGETVELOCITY = {2110, // 0 really far
            2110, // 1 really far
            1600, // 2 too close
            1710, // 3 near
            1800, // 4 medium
            2015, // 5  far
            1600, // 6 too close
            1710, // 7 medium
            1800, // 8 medium
            2015, // 9  far
            1710, // 10 near
            1800, // 11 medium
            2015, // 12 really far
            2110}; // 13 really far - ZONES indicated zone by number
    private enum shootingState{
        IDLE, START,INTAKE,MOTORSPINUP,FLINGER,END
    }
    private shootingState shooterState;
    public boolean startScoring = true;
    private double totalBalls = 3;

    public ShooterPIDF(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;

        flipper = new Flipper(hardwareMap);
        intake = new Intake(hardwareMap);
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        motor2 = hardwareMap.get(DcMotorEx.class, "shooterTwo");
        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        pidf = new PIDFController();

        scoreTimer = new Timer();
        shooterTimer = new Timer();

        shooterState = shootingState.IDLE;
    }

    public class PIDFController {
        private double integralSum = 0;
        private double lastError = 0;
        private long lastTime = System.nanoTime();

        public double calculate(double target, double current, double voltage) {
            double error = target - current;

            long now = System.nanoTime();
            double deltaTime = (now - lastTime) / 1e9;
            lastTime = now;

            integralSum += error * deltaTime;
            double derivative = (error - lastError) / deltaTime;
            lastError = error;

            double voltageComp = nominalVoltage / voltage;

            double feedforward = kF * target * voltageComp;

            return (kP * error)
                    + (kI * integralSum)
                    + (kD * derivative)
                    + feedforward;
        }

        public void reset() {
            integralSum = 0;
            lastError = 0;
            lastTime = System.nanoTime();
        }
    } // end of PIDFController()

    public void flywheelUpdatePower(double targetVelocity) {
        double ticksPerRev = 28;
        double targetTicksPerSecond =
                targetRPM * ticksPerRev / 60.0;

        double currentVelocity = motor.getVelocity();
        double batteryVoltage = batteryVoltageSensor.getVoltage();
        double currentRPM = currentVelocity * 60 / ticksPerRev;

        /*
        isShotReady = shotReady.isReady(
                targetRPM,
                currentRPM);
         */


        double power = pidf.calculate(
                targetTicksPerSecond,
                currentVelocity,
                batteryVoltage
        );

        motor.setPower(Math.max(-1.0, Math.min(1.0, power)));
        motor2.setPower(Math.max(-1.0, Math.min(1.0, power)));

        /*

        telemetry.addData("Battery Voltage", batteryVoltage);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentVelocity * 60 / ticksPerRev);
        telemetry.addData("Error", targetTicksPerSecond - currentVelocity);
        telemetry.addData("Motor Power", power);
        telemetry.addData("Shot Ready", isShotReady);

         */


    }


    public void setShooterVelocity(double velocity) {
        if (velocity == 0) {
            motorvelocity= NEARVELOCITY;
        } else if (velocity == 1) {
            motorvelocity = MEDIUMVELOCITY;
        } else if (velocity == 2) {
            motorvelocity = FARVELOCITY;
        } else if (velocity == 3) {
            motorvelocity = REALLYFARVELOCITY;
        } else {
            motorvelocity = velocity;
        }
    }

    public double calculateShooterVelocity(double range) {
        double velocity = RPM_50;

        if (range > 129) {
            velocity = RMP_130;
        } else if (range > 125) {
            velocity = RPM_126;
        } else if (range > 122) {
            velocity = RPM_123;
        } else if (range > 111) {
            velocity = RPM_112;
        } else if (range > 105) {
            velocity = RPM_106;
        } else if (range> 75){
            velocity = RPM_76;
        } else if (range > 62) {
            velocity = RPM_63;
        } else if (range > 58) {
            velocity = RPM_59;
        } else if (range > 50) {
            velocity = RPM_50;
        }

        // FORMULA??
        //velocity = 289.13317*Math.sin(0.0318839*range+2.92209)+2192.51129;

        return velocity;
    }


    public void spin(double velocity) {

        motorVoltage = 12 / hardwareMap.voltageSensor.iterator().next().getVoltage();

        motor.setVelocity(velocity*motorVoltage);
        motor2.setVelocity(velocity*motorVoltage);
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
                    motorVoltage = 12 / hardwareMap.voltageSensor.iterator().next().getVoltage();
                    //normalizedMotorVelocity = motorvelocity*motorVoltage;
                    normalizedMotorVelocity = motorvelocity;

                    motor2.setVelocity(normalizedMotorVelocity);
                    motor.setVelocity(normalizedMotorVelocity);

                    shooterTimer.resetTimer();
                    shooterState = shootingState.INTAKE;
                    intake.spin(1.0);
                    telemetry.addLine(String.format("SHOOTER UPDATE:START normalized velocity %6.1f",
                            normalizedMotorVelocity));
                    break;
                }
                case INTAKE: {
                    intake.spin(1.0);
                    // FIRST ball, startShootingProcess will be true, so give the intake MORE time
                    if (startShootingProcess && (shooterTimer.getElapsedTimeSeconds() > INTAKE_TIME_START)) {
                        shooterState = shootingState.MOTORSPINUP;
                        shooterTimer.resetTimer();
                        // NOT the first ball, startShootingProcess will be false, intake already moving, so give the intake LESS time
                    } else if (!startShootingProcess && (shooterTimer.getElapsedTimeSeconds() > INTAKE_TIME_CONTINUE)) {
                        shooterState = shootingState.MOTORSPINUP;
                        shooterTimer.resetTimer();
                    }
                    telemetry.addData("SHOOTER UPDATE","INTAKE");
                    break;
                }
                case MOTORSPINUP: {
                    if ((motor.getVelocity() > normalizedMotorVelocity - VELOCITY_LOWER_OFFSET) &&
                            (motor.getVelocity() < normalizedMotorVelocity + VELOCITY_UPPER_OFFSET)
                            || shooterTimer.getElapsedTimeSeconds() > JUST_SHOOT_IT) {
                        shooterTimer.resetTimer();
                        shooterState = shootingState.FLINGER;
                        flipper.up();
                    }
                    telemetry.addLine(String.format("SHOOTER UPDATE:MOTORSPINUP actual velocity %6.1f",
                            motor.getVelocity()));
                    break;
                }
                case FLINGER: {
                    if (shooterTimer.getElapsedTimeSeconds() > FLIPPER_DOWN) { // was 1.5
                        shooterState = shootingState.END;
                    } else if (shooterTimer.getElapsedTimeSeconds() > FLIPPER_UP) {  // was 1
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
