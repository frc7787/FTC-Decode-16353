package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class IndicatorLights {

    private double IDEALINTERVAL = 20;
    private double GOODINTERVAL = 40;
    private double BADINTERVAL = 30;

    private double IDEALANGLEINTERVAL = 4;
    private double GOODANGLEINTERVAL = 6;
    private double BADANGLEINTERVAL = 8;

    public enum targeting {GREEN, RED, OFF};

    LED targetLEDGreen;
    LED targetLEDRed;
    LED angleLEDGreen;
    LED angleLEDRed;

    public IndicatorLights(HardwareMap hardwareMap) {
        /*
        targetLEDGreen = hardwareMap.get(LED.class, "rangeLEDGreen");
        targetLEDRed = hardwareMap.get(LED.class, "rangeLEDRed");
        angleLEDGreen = hardwareMap.get(LED.class, "angleLEDGreen");
        angleLEDRed = hardwareMap.get(LED.class, "angleLEDRed");
         */
        // switch the GREEN and RED, here instead of hardware, because it seems to work ?
        targetLEDRed = hardwareMap.get(LED.class, "rangeLEDGreen");
        targetLEDGreen = hardwareMap.get(LED.class, "rangeLEDRed");
        angleLEDRed = hardwareMap.get(LED.class, "angleLEDGreen");
        angleLEDGreen = hardwareMap.get(LED.class, "angleLEDRed");
    } // end constructor

    public void display(targeting autoTargeting, Double angle, Double idealAngle) {

        if (autoTargeting == targeting.GREEN) {
            targetLEDGreen.on();
            targetLEDRed.off(); // Green for automatic targeting ON
        } else if (autoTargeting == targeting.RED){
            targetLEDGreen.off();
            targetLEDRed.on(); // Red for automatic targeting OFF
        } else {
            targetLEDGreen.off();
            targetLEDRed.off(); // Green for automatic targeting ON
        }


        if ((angle > idealAngle - IDEALANGLEINTERVAL) && (angle < idealAngle + IDEALANGLEINTERVAL)) {
            angleLEDGreen.on();
            angleLEDRed.off(); // Green for within ideal range
        } else if ((angle > idealAngle - GOODANGLEINTERVAL) && (angle < idealAngle + GOODANGLEINTERVAL)) {
            angleLEDGreen.off();  // Red for within good range
            angleLEDRed.on(); // Yellow doesn't work so well, so red means close but not quite
        } else if ((angle > idealAngle - BADANGLEINTERVAL) && (angle < idealAngle + BADANGLEINTERVAL)) {
            angleLEDGreen.off();
            angleLEDRed.off();
        } else {
            angleLEDGreen.off();
            angleLEDRed.off();
        }
    } // end display
} // end public class IndicatorLights
