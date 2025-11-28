package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebCamMechanismsSahaj;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous
public class AprilTagWebCamOpModeSahaj extends OpMode {

     AprilTagWebCamMechanismsSahaj aprilTagWebCam = new AprilTagWebCamMechanismsSahaj();


    @Override
    public void init() {
        aprilTagWebCam.init(hardwareMap, telemetry);
        aprilTagWebCam.setManualExposure(10, 6);


    }

    @Override
    public void loop() {
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

