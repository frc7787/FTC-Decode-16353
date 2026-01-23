package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Configurable

@Disabled
@TeleOp(name = "Flywheel Tuner Testing", group = "$")
public class FlywheelTunerTesting extends OpMode {
    public DcMotorEx flywheelMotor, flywheelMotor2;

    double[] listOfLowVelocities = {1900, 1940, 2190, 2270, 2380, 2400};

    double[] listOfHighVelocities = {1940, 2190, 2270, 2380, 2400, 2440};
    int highVelocityStepIndex = 1;
    int lowVelocityStepIndex = 1;
    public static double highVelocity = 1940;
    public static double lowVelocity = 1900;
    double curTargetVelocity = highVelocity;
    double F = 0;
    double P = 0;
    double[] stepSizes = {10.0, 1.0, 0.1, 0.01, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooterTwo");
        flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients2);






        telemetry.addLine("Init complete");

    }

    @Override
    public void loop() {
        // get all our gamepad commands
        // set target velocity
        // update telemetry

        if (gamepad1.yWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else { curTargetVelocity = highVelocity; }
        }
        if (gamepad1.xWasPressed()) {
            highVelocityStepIndex = (highVelocityStepIndex + 1) % listOfHighVelocities.length;
            highVelocity = listOfHighVelocities[highVelocityStepIndex];
            curTargetVelocity = highVelocity;
        }

        if (gamepad1.aWasPressed()) {
            lowVelocityStepIndex = (lowVelocityStepIndex + 1) % listOfLowVelocities.length;
            lowVelocity = listOfLowVelocities[lowVelocityStepIndex];
            curTargetVelocity = lowVelocity;
        }

        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }

        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        // set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        PIDFCoefficients pidfCoefficients2 = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients2);

        // set velocity
        flywheelMotor.setVelocity(curTargetVelocity);
        flywheelMotor2.setVelocity(curTargetVelocity);

        double curVelocity = flywheelMotor. getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("First, tune F for FeedForward", "%.4f (D-Pad L/R)", F);
        telemetry.addData("Then tune P", "%.4f (D-Pad U/D)", P);
        telemetry.addLine("Use PANELS to change the upper and lower velocity: 192.168.43.1:8001");
        telemetry.addLine("Y to Change Velocity");
        telemetry.addLine("B to Change Step Size");
        telemetry.addLine("X to change high Velocity");
        telemetry.addLine("A to change low Velocity");
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("High Velocity", highVelocity);
        telemetry.addData("Low Velocity", lowVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("-----------------------------");


        telemetry.addData("Step Size", "%.4f (B button)", stepSizes[stepIndex]);
    }
}
