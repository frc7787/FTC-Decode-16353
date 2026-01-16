package org.firstinspires.ftc.teamcode.Mechanisms;

import static java.lang.Thread.sleep;

import android.util.Size;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagWebCamMechanismsSahaj {

    private AprilTagProcessor aprilTagProcessor;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    private KalmanFilter xfilter, yfilter, zfilter;
    private KalmanFilter rangeFilter, bearingFilter, elevationFilter;
    private KalmanFilterParameters kalmanParams;

    public void init(HardwareMap hwMap, Telemetry telemetry)  {
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640 , 480));
        builder.addProcessor(aprilTagProcessor);


        visionPortal = builder.build();

        kalmanParams = new KalmanFilterParameters(0.05, 0.1);
        xfilter = new KalmanFilter(kalmanParams);
        yfilter = new KalmanFilter(kalmanParams);
        zfilter = new KalmanFilter(kalmanParams);

        rangeFilter = new KalmanFilter(kalmanParams);
        bearingFilter = new KalmanFilter(kalmanParams);
        elevationFilter = new KalmanFilter(kalmanParams);





    }

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();

    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {return;}
        if (detectedId.metadata!= null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (in)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY: %6.1f  %6.1f  %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (in, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        }
        else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f  (pixels)", detectedId.center.x, detectedId.center.y ));

        }



    }

    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection: detectedTags) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null;
    }
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }

    }

    public void setManualExposure(int exposureMS, int gain) {
        try {


            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);


            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);

        } catch (Exception e) {
            telemetry.addData("Error", "Failed to set camera exposure: " + e.getMessage());
        }
    }

    public AprilTagDetection getSmoothedTag(int id) {
        AprilTagDetection realTag = getTagBySpecificId(id);
        if (realTag != null) {
            xfilter.update(realTag.ftcPose.x, 0);
            yfilter.update(realTag.ftcPose.y, 0);
            zfilter.update(realTag.ftcPose.z, 0);
            rangeFilter.update(realTag.ftcPose.range, 0);
            bearingFilter.update(realTag.ftcPose.bearing, 0);
            elevationFilter.update(realTag.ftcPose.elevation, 0);

            telemetry.addLine("=== SMOOTHED KALMAN DATA ===");
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (cm, deg, deg)",
                    rangeFilter.getState(), bearingFilter.getState(), elevationFilter.getState()));
            telemetry.addLine("=== SMOOTHED KALMAN DATA ===");
            telemetry.addLine(String.format("XYZ Range %6.1f %6.1f %6.1f %6.1f cm)",
                    xfilter.getState(), yfilter.getState(), zfilter.getState(), rangeFilter.getState()));

            return realTag;
        }
        return null;
    }


    public double getSmoothedX() {
        return xfilter.getState();
    }

    public double getSmoothedY() {
        return yfilter.getState();
    }

    public double getSmoothedZ() {
        return zfilter.getState();
    }

    public double getSmoothedRange() {
        return rangeFilter.getState();
    }

    public double getSmoothedBearing() {
        return bearingFilter.getState();
    }

    public double getSmoothedElevation() {
        return elevationFilter.getState();
    }

}

