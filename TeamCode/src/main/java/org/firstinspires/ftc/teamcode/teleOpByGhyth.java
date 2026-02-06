package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.Gate;
import org.firstinspires.ftc.teamcode.Mechanisms.IndicatorLights;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;

import java.util.Timer;


@TeleOp
public class  teleOpByGhyth extends OpMode {
    private DcMotor backRightDrive, frontRightDrive, backLeftDrive, frontLeftDrive;

    private double shooterTargetVelocity;
    private boolean scoringThree = false;
    private boolean scoringOne = false;

    private String gatePlace;


    private boolean DEBUG = true;

    private Timer intakeSpinTimer;

    private double[] results;

    private double shooterVelocity = 0;

    private boolean lastOptions = false;
    private AprilTagSubsystem aprilTagSubsystem;
    private String shooterDistance;

    private double distanceAprilTag = 9999;
    private double angleAprilTag = 9999;

    private MecanumDriveBase mecanumDrive;
    private Intake intake;
    private Shooter shooter;
    private boolean AUTOSCORE = false;
    private Gate gate;
     private IndicatorLights indicatorLights;


    private IMU imu;

    public void init() {

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);
        mecanumDrive = new MecanumDriveBase(hardwareMap);


        intakeSpinTimer = new Timer();


        // drive motor init





        telemetry.addLine("Simple Field-Centric Initialized");
        telemetry.update();





    }

    @Override
    public void loop() {
        //Drive code

        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);

        mecanumDrive.driveFieldCentric(drive, strafe, turn);
        
       // Robot heading

        intake.spin(gamepad2.left_trigger - gamepad2.right_trigger) ;
        shooterVelocity = shooter.velocity();
        if (gamepad1.options) {
            mecanumDrive.resetImu();
        }
        if (gamepad1.triangleWasPressed()) {
            // near
            shooter.setShooterVelocity(0);
            shooter.spin(shooter.NEARVELOCITY);
            shooterDistance = "Near";
        } else if (gamepad1.squareWasPressed()) {
            shooter.spin(shooter.MEDIUMVELOCITY);
            // medi
            shooter.setShooterVelocity(1);

            shooterDistance = "Medium";
        } else if (gamepad1.circleWasPressed()) {
            // far
            shooter.setShooterVelocity(2);

            shooter.spin(shooter.FARVELOCITY);
            shooterDistance = "Far";
        } else if (gamepad1.crossWasPressed()) {
            // really fa
            shooter.spin(shooter.REALLYFARVELOCITY);
            shooter.setShooterVelocity(3);

            shooterDistance = "Really Far";
        }

        if (gamepad2.rightBumperWasPressed()) {
            if(shooterDistance == "Really Far"){
                shooter.spin(shooter.REALLYFARVELOCITY);
            } else if (shooterDistance == "Far") {
                shooter.spin(shooter.FARVELOCITY);
            } else if (shooterDistance == "Medium") {
               shooter.spin(shooter.MEDIUMVELOCITY);
            } else if (shooterDistance == "Near") {
                shooter.spin(shooter.NEARVELOCITY);
            }
        } else if (gamepad2.left_bumper) {
            if (shooterVelocity > 1000) {
                shooter.spin(0.0);
            } else {
                shooter.spin(-500);
            }
        } else if (gamepad2.leftBumperWasReleased()) {
            shooter.spin(0.0);
        }

        if (gamepad2.dpad_up) {
            gatePlace = "GateClosed";
            gate.closed();

        } else if (gamepad2.dpad_down) {
            gate.open();
            gatePlace = "GateOpen";
        }
// Please don't use strings for comparison, use a boolean instead - Tyler. S
//        if(gatePlace == "FlipperUp"){
//            intake.spin(1);
//
//        }

       /* if (gamepad1.leftBumperWasPressed()) {
            shooterTargetVelocity = shooterTargetVelocity - 5;
        } else if (gamepad1.rightBumperWasPressed()) {
            shooterTargetVelocity = shooterTargetVelocity + 5;
        }*/


        if (gamepad2.a && !scoringThree) {
            scoringThree = true;
            shooter.startScoring = true;    // reset shooter
        }

        if (scoringThree) {
            if (shooter.score(true, 3, telemetry)) {
                scoringThree = false;       // finished scoring all 3 balls
            }
        }

        if (gamepad2.x && !scoringOne) {
            scoringOne = true;
            shooter.startScoring = true;
        }

        if (scoringOne) {
            if (shooter.score(true, 1, telemetry)) {
                scoringOne = false;         // finished firing 1 ball
            }
        }
        if (DEBUG) {
            telemetry.addData("SHOOTER DISTANCE", shooterDistance);
            telemetry.addData("SHOOTER TARGET VELOCITY",shooterTargetVelocity);
            telemetry.addData("SHOOTER ACTUAL VELOCITY",shooterVelocity);

            telemetry.addData("RANGE",distanceAprilTag);
            telemetry.addData("ANGLE",angleAprilTag);
        }

    }
}



