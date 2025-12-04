package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.Flipper;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDriveBase;

@TeleOp(name = "TeleOpByTyler", group = "$")
public class TeleOpByTylerNov26 extends OpMode {

    private AprilTagSubsystem aprilTagSubsystem;

    private MecanumDriveBase mecanumDrive;
    private Intake intake;
    private Shooter shooter;

    private Flipper flipper;

    private boolean DEBUG = true;

    @Override public void init() {
        mecanumDrive = new MecanumDriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        flipper = new Flipper(hardwareMap);

        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);
    }

    @Override public void start() {

    }

    @Override public void loop() {
        double drive = -gamepad1.left_stick_y;
        drive *= Math.abs(drive);
        double strafe = gamepad1.left_stick_x;
        strafe *= Math.abs(strafe);
        double turn = gamepad1.right_stick_x;
        turn *= Math.abs(turn);

        mecanumDrive.driveRobotCentric(drive, strafe, turn);

        if (gamepad1.options) {
            mecanumDrive.resetImu();
        }

        intake.spin(gamepad2.left_trigger - gamepad2.right_trigger);

        if (gamepad2.rightBumperWasPressed()) {
            shooter.spin(2000);
        } else if (gamepad2.left_bumper) {
            if (shooter.velocity() > 1000) {
                shooter.spin(0.0);
            } else {
                shooter.spin(-500);
            }
        } else if (gamepad2.leftBumperWasReleased()) {
            shooter.spin(0.0);
        }

        if (gamepad2.dpad_up) {
            flipper.up();
        } else if (gamepad2.dpad_down) {
            flipper.down();
        }

        aprilTagSubsystem.update();

        if (DEBUG) {
            telemetry.addData("SHOOTER VELOCITY",shooter.velocity());
            telemetry.update();
        }

        aprilTagSubsystem.debug(telemetry,20);
    } // end loop
}

