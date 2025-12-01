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

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    private final KalmanFilter xFilter,
                               yFilter,
                               zFilter;

    private List<AprilTagDetection> detections;

    public AprilTagSubsystem(@NotNull HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
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
    }
}
