package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShotProfileConfig {

    // OFF
    public static double offRPM = 0;
    public static double offP = 0.00028;
    public static double offI = 0.0;
    public static double offD = 0.000018;
    public static double offF = 0.00011;

    // CLOSE
    public static double closeRPM = 1450;
    public static double closeP = 0.000009;
    public static double closeI = 0.0;
    public static double closeD = 0.0;
    public static double closeF = 0.00076;

    // MID
    public static double midRPM = 1540;
    public static double midP = 0.000009;
    public static double midI = 0.0;
    public static double midD = 0.0;
    public static double midF = 0.00076;

    // FAR
    public static double farRPM = 1780;
    public static double farP = 0.000009; // 0.0000005
    public static double farI = 0.0;
    public static double farD = 0.0;
    public static double farF = 0.00076; // 0.0008154

    // POWER SHOT
    public static double powerRPM = 1940;
    public static double powerP = 0.000009;
    public static double powerI = 0.0;
    public static double powerD = 0.0;
    public static double powerF = 0.00076;
}

