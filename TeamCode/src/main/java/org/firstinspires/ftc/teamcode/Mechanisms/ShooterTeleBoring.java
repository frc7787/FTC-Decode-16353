package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class ShooterTeleBoring {
    private TelemetryManager telemetryM;

    // FLYWHEEL configurable PIDF variables

    public static double PIDF_F_BORING = 11.19;
    public static double PIDF_P_BORING = 60.0;
    PIDFCoefficients pidfCoefficientsBoring = new PIDFCoefficients(PIDF_P_BORING,0,0,PIDF_F_BORING);

    public static double kP = 0.0000005;  // 0.0000005
    public static double kI = 0.0000;
    public static double kD = 0.0000;
    public static double kF = 0.0008154; // 0.0008154

    public static double targetRPM = 1500;

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

    //public static double PIDF_F = 11.19;
    //public static double PIDF_P = 60.0;

    //PIDFCoefficients pidfCoefficients = new PIDFCoefficients(PIDF_P, 0, 0, PIDF_F);

    private Gate gate;
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

    public enum ShotProfile {
        OFF,
        CLOSE,
        MID,
        FAR,
        POWER
    }

    // THESE VARIABLES ARE FOR THE AUTOMATIC APRIL TAG TARGETING range and flywheel RPM

    public static double RMP_130 = 1960;
    public static double RPM_126 = 1930;
    public static double RPM_123 = 1900;
    public static double RPM_112 = 1820;
    public static double RPM_106 = 1760;
    public static double RPM_76 = 1610;
    public static double RPM_63 = 1540;
    public static double RPM_59 = 1510;
    public static double RPM_50 = 1440;
    public static double RPM_46 = 1360;
    public static double RPM_42 = 1340;
    public static double RPM_AUDIENCE = 1790; // was 1760 @12.9V; was 2290/2240 new wheel
    public static double RPM_GOAL = 1400; // was 1940 old wheel


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

    public ShooterTeleBoring(HardwareMap hardwareMap) {

        this.hardwareMap = hardwareMap;

        gate = new Gate(hardwareMap);
        intake = new Intake(hardwareMap);
        motor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        motor.setDirection(DcMotorEx.Direction.REVERSE);
        //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficientsBoring);

        motor2 = hardwareMap.get(DcMotorEx.class, "shooterTwo");
        motor2.setDirection(DcMotorEx.Direction.FORWARD);
        //motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficientsBoring);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        pidf = new PIDFController();

        scoreTimer = new Timer();
        shooterTimer = new Timer();

        shooterState = shootingState.IDLE;

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public class PIDFController {

        private double integralSum = 0;
        private double lastError = 0;
        private long lastTime = System.nanoTime();

        private ShotProfile formerProfile = ShotProfile.MID;

        public double calculate(
                ShotProfile profile,
                double targetVelocity,
                double currentVelocity,
                double voltage
        ) {
            double target;
            double kP, kI, kD, kF;

            telemetryM.addData("FORMER PROFILE:", formerProfile);
            telemetryM.addData("CURRENT PROFILE", profile);

            if (formerProfile != profile) {
                reset();
                formerProfile = profile;
            }

            switch (profile) {
                case OFF:
                    target = ShotProfileConfig.offRPM;
                    kP = ShotProfileConfig.offP;
                    kI = ShotProfileConfig.offI;
                    kD = ShotProfileConfig.offD;
                    kF = ShotProfileConfig.offF;
                    break;
                case CLOSE:
                    target = ShotProfileConfig.closeRPM;
                    kP = ShotProfileConfig.closeP;
                    kI = ShotProfileConfig.closeI;
                    kD = ShotProfileConfig.closeD;
                    kF = ShotProfileConfig.closeF;
                    break;

                case MID:
                    target = ShotProfileConfig.midRPM;
                    kP = ShotProfileConfig.midP;
                    kI = ShotProfileConfig.midI;
                    kD = ShotProfileConfig.midD;
                    kF = ShotProfileConfig.midF;
                    break;

                case FAR:
                    target = ShotProfileConfig.farRPM;
                    kP = ShotProfileConfig.farP;
                    kI = ShotProfileConfig.farI;
                    kD = ShotProfileConfig.farD;
                    kF = ShotProfileConfig.farF;
                    break;

                default: // POWER
                    target = ShotProfileConfig.powerRPM;
                    kP = ShotProfileConfig.powerP;
                    kI = ShotProfileConfig.powerI;
                    kD = ShotProfileConfig.powerD;
                    kF = ShotProfileConfig.powerF;
                    break;
            }

            double ticksPerRev = 28;
            double targetTicksPerSec = target * ticksPerRev / 60.0;

            double error = targetTicksPerSec - currentVelocity;

            long now = System.nanoTime();
            double dt = (now - lastTime) / 1e9;
            lastTime = now;

            integralSum += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            double voltageComp = ShooterPIDF.nominalVoltage / voltage;

            telemetryM.addData("TARGET:", target);
            telemetryM.addData("TargetTicksPerSec:", targetTicksPerSec);
            telemetryM.addData("Current Velocity", currentVelocity);
            telemetryM.addData("ERROR:", error);

            return (kP * error)
                    + (kI * integralSum)
                    + (kD * derivative)
                    + (kF * targetTicksPerSec * voltageComp);
        }

        public void reset() {
            integralSum = 0;
            lastError = 0;
            lastTime = System.nanoTime();
        }
    }
    // end of PIDFController()

    public void flywheelUpdatePower(ShotProfile currentProfile, double targetVelocity) {
        double ticksPerRev = 28;
        double targetTicksPerSecond =
                targetVelocity * ticksPerRev / 60.0;
        // targetRPM * ticksPerRev / 60.0;

        double currentVelocity = motor.getVelocity();
        double batteryVoltage = 12 / batteryVoltageSensor.getVoltage();
        double currentRPM = currentVelocity * 60 / ticksPerRev;

        /*
        isShotReady = shotReady.isReady(
                targetRPM,
                currentRPM);
         */


        // FOR NOW, take out the targetTicksPerSecond calculated from PARAMETER targetVelocity
        // and just use the PROFILE targetVelocity
        double power = pidf.calculate(
                currentProfile,
                targetVelocity,
                currentVelocity,
                batteryVoltage
        );

        telemetryM.addData("POWER:", power);
        telemetryM.addData("Battery:", batteryVoltage);
        telemetryM.addData("Target Velocity:", targetVelocity);
        telemetryM.addData("Current Velocity", currentRPM);
        telemetryM.update();

        /*
        motor.setPower(Math.max(-1.0, Math.min(1.0, power)));
        motor2.setPower(Math.max(-1.0, Math.min(1.0, power)));
         */

        //motor.setVelocity(targetVelocity*batteryVoltage);
        //motor2.setVelocity(targetVelocity*batteryVoltage);
        motor.setVelocity(targetVelocity);
        motor2.setVelocity(targetVelocity);


        /*

        telemetry.addData("Battery Voltage", batteryVoltage);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.addData("Current RPM", currentVelocity * 60 / ticksPerRev);
        telemetry.addData("Error", targetTicksPerSecond - currentVelocity);
        telemetry.addData("Motor Power", power);
        telemetry.addData("Shot Ready", isShotReady);

         */


    } // end flywheelUpdatePower

    public double getTargetRPM(ShotProfile profile) {
        switch (profile) {
            case CLOSE: return ShotProfileConfig.closeRPM;
            case MID: return ShotProfileConfig.midRPM;
            case FAR: return ShotProfileConfig.farRPM;
            default: return ShotProfileConfig.powerRPM;
        }
    } // end getTargetRPM

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
        double velocity = RPM_76;

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
        } else if (range > 49) {
            velocity = RPM_50;
        } else if (range > 45) {
            velocity = RPM_46;
        } else if (range > 41) {
            velocity = RPM_42;
        } else {
            velocity = RPM_42 - 40;
        }

        // FORMULA??
        //velocity = 289.13317*Math.sin(0.0318839*range+2.92209)+2192.51129;

        return velocity;
    }

    public ShotProfile calculateShooterProfile(double range) {
        ShotProfile profile = ShotProfile.MID;

        if (range<54) {
            profile = ShotProfile.CLOSE;
        } else if (range <72) {
            profile = ShotProfile.MID;
        } else if (range < 105) {
            profile = ShotProfile.FAR;
        } else {
            profile = ShotProfile.POWER;
        }



        // FORMULA??
        //velocity = 289.13317*Math.sin(0.0318839*range+2.92209)+2192.51129;

        return profile;
    }


    public void spin(double velocity) {

        motorVoltage = 12 / hardwareMap.voltageSensor.iterator().next().getVoltage();

        motor.setVelocity(velocity*motorVoltage);
        motor2.setVelocity(velocity*motorVoltage);
    }

    public double velocity() {
        return motor.getVelocity();
    }

    public boolean update(boolean startShootingProcess, boolean cancelShootingProcess, Telemetry telemetry) {

        if (cancelShootingProcess) {
            intake.spin(0);
            gate.closed(); // TODO I only changed the method name, the logic might still need to be updated
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
                    //intake.spin(1.0);
                    telemetry.addLine(String.format("SHOOTER UPDATE:START normalized velocity %6.1f",
                            normalizedMotorVelocity));
                    break;
                }
                case INTAKE: {
                    //intake.spin(1.0);
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
                        intake.spin(1.0);
                        //gate.open(); // TODO I only changed the method name, the logic might still need to be updated
                    }
                    telemetry.addLine(String.format("SHOOTER UPDATE:MOTORSPINUP actual velocity %6.1f",
                            motor.getVelocity()));
                    break;
                }
                case FLINGER: {
                    if (shooterTimer.getElapsedTimeSeconds() > FLIPPER_DOWN) { // was 1.5
                        shooterState = shootingState.END;
                    } else if (shooterTimer.getElapsedTimeSeconds() > FLIPPER_UP) {  // was 1
                        //gate.closed(); // TODO I only changed the method name, the logic might still need to be updated
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
            intake.spin(0.0);
            gate.open();
            this.update(true,false, telemetry); // start the shooting update process, with "true" for shootingstate START
        } else if (this.update(false,false, telemetry)) {
            telemetry.addData("SHOOTER SCORE", "update true, so minus one ball");
            totalBalls = totalBalls - 1;
            if (totalBalls == 0) {
                telemetry.addData("SHOOTER SCORE", "total balls equals ZERO");
                startScoring = true;
                gate.closed(); // only close the gate after firing ALL the balls
                return true; // finished firing all balls, return true for "score"
            } else {
                this.update(true, false, telemetry); // start the shooting update process, with "true" for shootingstate START
                return false; // if not finished firing all balls, return false for "score"
            }
        }
        return false;
    } // end score
}

