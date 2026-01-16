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
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.jetbrains.annotations.NotNull;

import java.util.List;
import java.util.concurrent.TimeUnit;

public class AprilTagSubsystem {

    // Configuration

    private static final KalmanFilterParameters KALMAN_FILTER_PARAMETERS = new KalmanFilterParameters(0.05, 0.1);
    private static final Size CAMERA_RESOLUTION = new Size(640, 480);

    private static final boolean DRAW_TAG_ID = true;
    private static final boolean DRAW_TAG_OUTLINE = true;
    private static final boolean DRAW_AXES = true;
    private static final boolean DRAW_CUBE_PROJECTION = true;
    private static final DistanceUnit OUTPUT_DISTANCE_UNIT = DistanceUnit.INCH;
    private static final AngleUnit OUTPUT_ANGLE_UNIT = AngleUnit.DEGREES;

    private static final String WEBCAM_NAME = "Webcam 1";

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    private final KalmanFilter xFilter,
                               yFilter,
                               zFilter,
                               rangeFilter,
                               bearingFilter,
                               elevationFilter;

    private List<AprilTagDetection> detections;

    public AprilTagSubsystem(@NotNull HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(DRAW_TAG_ID)
                .setDrawTagOutline(DRAW_TAG_OUTLINE)
                .setDrawAxes(DRAW_AXES)
                .setDrawCubeProjection(DRAW_CUBE_PROJECTION)
                .setOutputUnits(OUTPUT_DISTANCE_UNIT, OUTPUT_ANGLE_UNIT)
                .setCameraPose(cameraPosition, cameraOrientation)
                // first two numbers are focal length, next two are principal point
                .setLensIntrinsics(622.001f, 622.001, 319.803f, 241.251f) // Logitech C920 from teamwebcamcalibrations
                //.setLensIntrinsics(660.750, 660.75, 323.034, 230.681) // C615 measured kk Dec 5 2023
                //.setLensIntrinsics(822.317f, 822.317f, 319.495f,242.502f" // C270 from teamwebcamcalibrations
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, WEBCAM_NAME))
                .setCameraResolution(CAMERA_RESOLUTION)
                .addProcessor(aprilTagProcessor)
                .build();

        xFilter = new KalmanFilter(KALMAN_FILTER_PARAMETERS);
        yFilter = new KalmanFilter(KALMAN_FILTER_PARAMETERS);
        zFilter = new KalmanFilter(KALMAN_FILTER_PARAMETERS);

        rangeFilter = new KalmanFilter(KALMAN_FILTER_PARAMETERS);
        bearingFilter = new KalmanFilter(KALMAN_FILTER_PARAMETERS);
        elevationFilter = new KalmanFilter(KALMAN_FILTER_PARAMETERS);

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
        //xFilter.update(detection.ftcPose.x, 0);
        //yFilter.update(detection.ftcPose.y, 0);
        //zFilter.update(detection.ftcPose.z, 0);
        // use Kalman filter for ROBOTPOSE, but store it in the AprilTagDetection under AAprilTagPoseFtc
        xFilter.update(detection.robotPose.getPosition().x, 0);
        yFilter.update(detection.robotPose.getPosition().y, 0);
        zFilter.update(detection.robotPose.getPosition().z, 0);

        rangeFilter.update(detection.ftcPose.range, 0);
        bearingFilter.update(detection.ftcPose.bearing, 0);
        elevationFilter.update(detection.ftcPose.elevation, 0);
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
                        rangeFilter.getState(),
                        bearingFilter.getState(),
                        elevationFilter.getState()


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

            telemetry.addLine("APRIL TAG SUBSYSTEM Debug");
            telemetry.addLine("X");
            telemetry.addData("Raw", "%.2f", raw.ftcPose.x);
            telemetry.addData("Filtered", "%.2f", filtered.ftcPose.x);
            telemetry.addLine("Y");
            telemetry.addData("Raw", "%.2f", raw.ftcPose.y);
            telemetry.addData("Filtered", "%.2f", filtered.ftcPose.y);
            telemetry.addLine("Z");
            telemetry.addData("Raw", "%.2f", raw.ftcPose.z);
            telemetry.addData("Filtered", "%.2f", filtered.ftcPose.z);

            //new added kalman telemetry
            telemetry.addLine("Range");
            telemetry.addData("Raw", "%.2f", raw.ftcPose.range);
            telemetry.addData("Filtered", "%.2f", filtered.ftcPose.range);

            telemetry.addLine("Bearing");
            telemetry.addData("Raw", "%.2f", raw.ftcPose.bearing);
            telemetry.addData("Filtered", "%.2f", filtered.ftcPose.bearing);

            telemetry.addLine("Elevation");
            telemetry.addData("Raw", "%.2f", raw.ftcPose.elevation);
            telemetry.addData("Filtered", "%.2f", filtered.ftcPose.elevation);

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
            //distanceToAprilTag = Math.hypot(filtered.ftcPose.x, filtered.ftcPose.y); // actually use the Kalman Filtered RANGE!!!
           // distanceToAprilTag = Math.hypot(distanceToAprilTag, filtered.ftcPose.z);
            distanceToAprilTag = filtered.ftcPose.range;
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

    public void setManualExposure(Telemetry telemetry , int exposureMS, int gain,int whiteBalance) {
        try {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);

            WhiteBalanceControl whiteBalanceControl = visionPortal.getCameraControl(WhiteBalanceControl.class);
            whiteBalanceControl.setWhiteBalanceTemperature(whiteBalance);
            sleep(20);

        } catch (Exception e) {
            telemetry.addData("Error", "Failed to set camera exposure: " + e.getMessage());
        }
    }
}
