package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class mecanumDrive {

    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;
    public void init() {

        frontLeft = HardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = HardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = HardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = HardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {

    }
}
