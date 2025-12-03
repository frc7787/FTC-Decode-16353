package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.Mechanisms.MecanumDriveBase;

@TeleOp(name = "TeleOpByTyler", group = "$")
public class TeleOpByTylerNov26 extends OpMode {

    private AprilTagSubsystem aprilTagSubsystem;

    private MecanumDriveBase mecanumDrive;
    private Intake intake;
    private Shooter shooter;

    @Override public void init() {
        mecanumDrive = new MecanumDriveBase(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);

        aprilTagSubsystem = new AprilTagSubsystem(hardwareMap);
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

        if (gamepad2.left_bumper) {
            shooter.spin(1.0);
        } else if (gamepad2.right_bumper) {
            shooter.spin(-1.0);
        } else {
            shooter.spin(0.0);
        }

        aprilTagSubsystem.update();

        aprilTagSubsystem.debug(telemetry,20);
    } // end loop
}

