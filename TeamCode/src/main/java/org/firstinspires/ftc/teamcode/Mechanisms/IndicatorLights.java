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

    LED rangeLEDGreen;
    LED rangeLEDRed;
    LED angleLEDGreen;
    LED angleLEDRed;

    public IndicatorLights(HardwareMap hardwareMap) {
        rangeLEDGreen = hardwareMap.get(LED.class, "rangeLEDGreen");
        rangeLEDRed = hardwareMap.get(LED.class, "rangeLEDRed");
        angleLEDGreen = hardwareMap.get(LED.class, "angleLEDGreen");
        angleLEDRed = hardwareMap.get(LED.class, "angleLEDRed");
    } // end constructor

    public void display(Double range, Double idealRange, Double angle, Double idealAngle) {
        if ((range > idealRange - IDEALINTERVAL) && (range < idealRange + IDEALINTERVAL)) {
            rangeLEDGreen.on();
            rangeLEDRed.off(); // Green for within ideal range
        } else if ((range > idealRange - GOODINTERVAL) && (range < idealRange + GOODINTERVAL)) {
            rangeLEDGreen.off();  // Red for within good range
            rangeLEDRed.on(); // Yellow doesn't work so well, therefore only 2 ranges
        } else if ((range > idealRange - BADINTERVAL) && (range < idealRange + BADINTERVAL)) {
            rangeLEDGreen.off();
            rangeLEDRed.off();
        } else {
            rangeLEDGreen.off();
            rangeLEDRed.off();
        }

        if ((angle > idealAngle - IDEALANGLEINTERVAL) && (angle < idealAngle + IDEALANGLEINTERVAL)) {
            angleLEDGreen.on();
            angleLEDRed.off(); // Green for within ideal range
        } else if ((angle > idealAngle - GOODANGLEINTERVAL) && (angle < idealAngle + GOODANGLEINTERVAL)) {
            angleLEDGreen.off();  // Red for within good range
            angleLEDRed.on(); // Yellow doesn't work so well, therefore only 2 ranges
        } else if ((angle > idealAngle - BADANGLEINTERVAL) && (angle < idealAngle + BADANGLEINTERVAL)) {
            angleLEDGreen.off();
            angleLEDRed.off();
        } else {
            angleLEDGreen.off();
            angleLEDRed.off();
        }
    } // end display
} // end public class IndicatorLights
