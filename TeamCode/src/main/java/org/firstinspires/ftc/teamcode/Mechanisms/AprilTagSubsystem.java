package org.firstinspires.ftc.teamcode.Mechanisms;

import android.util.Size;

import com.pedropathing.control.KalmanFilter;
import com.pedropathing.control.KalmanFilterParameters;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.jetbrains.annotations.NotNull;

import java.util.List;

public class AprilTagSubsystem {

    // Configuration

    private static final KalmanFilterParameters KALMAN_FILTER_PARAMETERS = new KalmanFilterParameters(0.05, 0.1);
    private static final Size CAMERA_RESOLUTION = new Size(640, 480);

    private static final boolean DRAW_TAG_ID = true;
    private static final boolean DRAW_TAG_OUTLINE = true;
    private static final boolean DRAW_AXES = true;
    private static final boolean DRAW_CUBE_PROJECTION = true;
    private static final DistanceUnit OUTPUT_DISTANCE_UNIT = DistanceUnit.CM;
    private static final AngleUnit OUTPUT_ANGLE_UNIT = AngleUnit.DEGREES;

    private static final String WEBCAM_NAME = "Webcam 1";

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    private final KalmanFilter xFilter,
                               yFilter,
                               zFilter;

    private List<AprilTagDetection> detections;

    public AprilTagSubsystem(@NotNull HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(DRAW_TAG_ID)
                .setDrawTagOutline(DRAW_TAG_OUTLINE)
                .setDrawAxes(DRAW_AXES)
                .setDrawCubeProjection(DRAW_CUBE_PROJECTION)
                .setOutputUnits(OUTPUT_DISTANCE_UNIT, OUTPUT_ANGLE_UNIT)
                .setLensIntrinsics(622.001f, 622.001, 319.803f, 241.251f) // from teamwebcamcalibrations
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(CAMERA_RESOLUTION)
                .addProcessor(aprilTagProcessor)
                .build();

        xFilter = new KalmanFilter(KALMAN_FILTER_PARAMETERS);
        yFilter = new KalmanFilter(KALMAN_FILTER_PARAMETERS);
        zFilter = new KalmanFilter(KALMAN_FILTER_PARAMETERS);

        visionPortal.resumeStreaming();
    }

    public void resume(boolean startLiveView) {
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            return;
        }

        visionPortal.resumeStreaming();
        if (startLiveView) {
            visionPortal.resumeLiveView();
        }
    }

    public void update() {
        detections = aprilTagProcessor.getDetections();
    }

    public boolean tagDetectedWithId(int id) {
        for (AprilTagDetection detection: detections) {
            if (detection.id == id) {
                return true;
            }
        }
        return false;
    }

    @NotNull public AprilTagDetection tagWithId(int id) throws IllegalArgumentException {
        for (AprilTagDetection detection: detections) {
            if (detection.id == id) {
                return filter(detection);
            }
        }
        throw new IllegalArgumentException("No Detection Found With Id: " + id);
    }

    @NotNull public AprilTagDetection tagWithIdRaw(int id) throws IllegalArgumentException {
        for (AprilTagDetection detection: detections) {
            if (detection.id == id) {
                return detection;
            }
        }
        throw new IllegalArgumentException("No Detection Found With Id: " + id);
    }

    @NotNull private AprilTagDetection filter(@NotNull AprilTagDetection detection) {
        xFilter.update(detection.ftcPose.x, 0);
        yFilter.update(detection.ftcPose.y, 0);
        zFilter.update(detection.ftcPose.z, 0);
        return new AprilTagDetection(
                detection.id,
                detection.hamming,
                detection.decisionMargin,
                detection.center,
                detection.corners,
                detection.metadata,
                new AprilTagPoseFtc(
                        xFilter.getState(),
                        yFilter.getState(),
                        zFilter.getState(),
                        detection.ftcPose.yaw,
                        detection.ftcPose.pitch,
                        detection.ftcPose.roll,
                        detection.ftcPose.range,
                        detection.ftcPose.bearing,
                        detection.ftcPose.elevation
                ),
                detection.rawPose,
                detection.robotPose,
                detection.frameAcquisitionNanoTime
        );
    }

    public void stop() {
        visionPortal.stopLiveView();
        visionPortal.stopStreaming();
    }

    public void debug(Telemetry telemetry, int id) {
        if (tagDetectedWithId(id)) {
            /*
             * Safety: Because tagDetectedWithId is called by this point, tagWidthRawId and
             * tagWithId cannot be null
             */
            final AprilTagDetection raw = tagWithIdRaw(id);
            final AprilTagDetection filtered = tagWithId(id);

            telemetry.addLine("X");
            telemetry.addData("Raw", "%.2f", raw.ftcPose.x);
            telemetry.addData("Filtered", "%.2f", filtered.ftcPose.x);
            telemetry.addLine("Y");
            telemetry.addData("Raw", "%.2f", raw.ftcPose.y);
            telemetry.addData("Filtered", "%.2f", filtered.ftcPose.y);
            telemetry.addLine("Z");
            telemetry.addData("Raw", "%.2f", raw.ftcPose.z);
            telemetry.addData("Filtered", "%.2f", filtered.ftcPose.z);

        } else {
            telemetry.addLine("No tag detected with id: " + id);
        }
    } // end debug

    public double[] angleANDdistance(int id) {
        double distanceToAprilTag = 9999;
        double angleToAprilTag = 9999;
        final AprilTagDetection filtered;
        double[] results = new double[2];

        if (tagDetectedWithId(id)) {
            filtered = tagWithId(id);
            distanceToAprilTag = Math.hypot(filtered.ftcPose.x, filtered.ftcPose.y); // actually use the Kalman Filtered RANGE!!!
            distanceToAprilTag = Math.hypot(distanceToAprilTag, filtered.ftcPose.z);
            angleToAprilTag = filtered.ftcPose.bearing;
            results[0] = distanceToAprilTag;
            results[1] = angleToAprilTag;
        }

        return results;
    }

    public double angle(int id) {
        double angleToAprilTag = 9999;
        final AprilTagDetection filtered;

        if (tagDetectedWithId(id)) {
            filtered = tagWithId(id);
            angleToAprilTag = filtered.ftcPose.bearing;
        }

        return angleToAprilTag;
    }
}
