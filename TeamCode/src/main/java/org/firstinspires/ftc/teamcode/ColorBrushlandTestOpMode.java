package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp (name = "ColorBrushlandTestOpMode")
public class ColorBrushlandTestOpMode extends LinearOpMode {

    @Override public void runOpMode() {
        //RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "colorBrushland");
        DigitalChannel pin0 = hardwareMap.digitalChannel.get("digital0");
        DigitalChannel pin1 = hardwareMap.digitalChannel.get("digital1");

        waitForStart();
        while (opModeIsActive()) {
            // BRUSHLAND LABS color sensor as a simple RGB
            // read all 3 color channels in one I2C transmission:
            //NormalizedRGBA colors = sensor.getNormalizedColors();
            //telemetry.addData("rgb: ", colors.red + " " + colors.blue + " " + colors.green);
            //telemetry.update();

            // BRUSHLAND LABS digital sensor, you must configure it FIRST as a color sensor with HSV thresholds
            telemetry.addData("digital 0", pin0.getState());
            telemetry.addData("digital 1", pin1.getState());
            telemetry.update();
        }

    }
}
