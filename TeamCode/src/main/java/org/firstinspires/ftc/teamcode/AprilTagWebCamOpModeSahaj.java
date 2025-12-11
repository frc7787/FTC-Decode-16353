package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebCamMechanismsSahaj;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
@Disabled
public class AprilTagWebCamOpModeSahaj extends OpMode {

     AprilTagWebCamMechanismsSahaj aprilTagWebCam = new AprilTagWebCamMechanismsSahaj();
     private boolean exposureSet = false;


    @Override
    public void init() {
        aprilTagWebCam.init(hardwareMap, telemetry);


    }

    @Override
    public void loop() {

        if (!exposureSet) {
            aprilTagWebCam.setManualExposure(10, 6);
            exposureSet = true;
        }
        // update the vision portal
        aprilTagWebCam.update();
        List<AprilTagDetection> detectedTags = aprilTagWebCam.getDetectedTags();

        if (detectedTags.isEmpty()) {
            telemetry.addLine("No tags detected");
        } else {
            for (AprilTagDetection tag : detectedTags) {
                aprilTagWebCam.displayDetectionTelemetry(tag);
                aprilTagWebCam.getSmoothedTag(tag.id);
            }
        }
        telemetry.update();
    }
}

