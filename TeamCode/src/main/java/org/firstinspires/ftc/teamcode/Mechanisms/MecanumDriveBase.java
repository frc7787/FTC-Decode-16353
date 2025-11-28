package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.*;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;


public class MecanumDriveBase {

    // -------------------------
    // Hardware


    private final DcMotorEx frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor;

    private final IMU imu;

    // -------------------------
    // Configuration

    private static final String FRONT_LEFT_MOTOR_NAME  = "frontLeftDrive";
    private static final String FRONT_RIGHT_MOTOR_NAME = "frontRightDrive";
    private static final String BACK_LEFT_MOTOR_NAME   = "backLeftDrive";
    private static final String BACK_RIGHT_MOTOR_NAME  = "backRightDrive";
    private static final String IMU_NAME = "imu";

    private static final ZeroPowerBehavior FRONT_LEFT_MOTOR_ZERO_POWER_BEHAVIOR  = BRAKE;
    private static final ZeroPowerBehavior FRONT_RIGHT_MOTOR_ZERO_POWER_BEHAVIOR = BRAKE;
    private static final ZeroPowerBehavior BACK_LEFT_MOTOR_ZERO_POWER_BEHAVIOR   = BRAKE;
    private static final ZeroPowerBehavior BACK_RIGHT_MOTOR_ZERO_POWER_BEHAVIOR  = BRAKE;

    private static final Direction FRONT_LEFT_MOTOR_DIRECTION  = Direction.REVERSE;
    private static final Direction FRONT_RIGHT_MOTOR_DIRECTION = Direction.FORWARD;
    private static final Direction BACK_LEFT_MOTOR_DIRECTION   = Direction.REVERSE;
    private static final Direction BACK_RIGHT_MOTOR_DIRECTION  = Direction.FORWARD;

    private final IMU.Parameters IMU_PARAMETERS = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
            )
    );

    public MecanumDriveBase(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, FRONT_LEFT_MOTOR_NAME);
        frontRightMotor = hardwareMap.get(DcMotorEx.class, FRONT_RIGHT_MOTOR_NAME);
        backLeftMotor = hardwareMap.get(DcMotorEx.class, BACK_LEFT_MOTOR_NAME);
        backRightMotor = hardwareMap.get(DcMotorEx.class, BACK_RIGHT_MOTOR_NAME);

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(IMU_PARAMETERS);

        frontLeftMotor.setZeroPowerBehavior(FRONT_LEFT_MOTOR_ZERO_POWER_BEHAVIOR);
        frontRightMotor.setZeroPowerBehavior(FRONT_RIGHT_MOTOR_ZERO_POWER_BEHAVIOR);
        backLeftMotor.setZeroPowerBehavior(BACK_LEFT_MOTOR_ZERO_POWER_BEHAVIOR);
        backRightMotor.setZeroPowerBehavior(BACK_RIGHT_MOTOR_ZERO_POWER_BEHAVIOR);

        frontLeftMotor.setDirection(FRONT_LEFT_MOTOR_DIRECTION);
        frontRightMotor.setDirection(FRONT_RIGHT_MOTOR_DIRECTION);
        backLeftMotor.setDirection(BACK_LEFT_MOTOR_DIRECTION);
        backRightMotor.setDirection(BACK_RIGHT_MOTOR_DIRECTION);
    }

    public void driveFieldCentric(double drive, double strafe, double turn) {
        double yawRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double thetaRadians = Math.atan2(drive, strafe) - yawRadians;

        double power = Math.hypot(drive, strafe);

        double sinTheta = Math.sin(thetaRadians - Math.PI / 4.0);
        double cosTheta = Math.cos(thetaRadians - Math.PI / 4.0);
        double max = Math.max(Math.abs(sinTheta), Math.abs(cosTheta));

        double frontLeftPower  = power * cosTheta / max + turn;
        double frontRightPower = power * sinTheta / max - turn;
        double backLeftPower   = power * sinTheta / max + turn;
        double backRightPower  = power * cosTheta / max - turn;

        double turnMagnitude = Math.abs(turn);

        if (power + turnMagnitude > 1.0) {
            frontLeftPower  /= turnMagnitude;
            frontRightPower /= turnMagnitude;
            backLeftPower   /= turnMagnitude;
            backRightPower  /= turnMagnitude;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void driveRobotCentric(double drive, double strafe, double turn ) {
        double thetaRadians = Math.atan2(drive, strafe);

        double power = Math.hypot(drive, strafe);

        double sinTheta = Math.sin(thetaRadians - Math.PI / 4.0);
        double cosTheta = Math.cos(thetaRadians - Math.PI / 4.0);
        double max = Math.max(Math.abs(sinTheta), Math.abs(cosTheta));

        double frontLeftPower  = power * cosTheta / max + turn;
        double frontRightPower = power * sinTheta / max - turn;
        double backLeftPower   = power * sinTheta / max + turn;
        double backRightPower  = power * cosTheta / max - turn;

        double turnMagnitude = Math.abs(turn);

        if (power + turnMagnitude > 1.0) {
            frontLeftPower  /= turnMagnitude;
            frontRightPower /= turnMagnitude;
            backLeftPower   /= turnMagnitude;
            backRightPower  /= turnMagnitude;
        }

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    public void resetImu() {
        imu.resetYaw();
    }
}
