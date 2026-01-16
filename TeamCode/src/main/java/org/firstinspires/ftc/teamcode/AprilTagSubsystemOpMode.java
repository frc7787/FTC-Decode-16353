package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagSubsystem;
import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagWebCamMechanismsSahaj;
import org.firstinspires.ftc.teamcode.Mechanisms.IndicatorLights;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name="AprilTag Subsystem OpMode", group = "Concept")
public class AprilTagSubsystemOpMode extends OpMode {

    private AprilTagSubsystem aprilTagWebcam;
    private boolean exposureSet = false;

    private IndicatorLights indicatorLights;

    AprilTagDetection currentTagValues;

    private boolean currentTagDetected;
    private double distanceAprilTag = 9999;
    private double angleAprilTag = 9999;

    private double shooterTargetVelocity = 9999;

    private double[] shootingAngles = {0,5,10,10,10,10,5,5,5,5,-5,0,5,10}; // 13 indicated zone by number

    public double[] TARGETVELOCITY = {2110, // 0 really far
            2110, // 1 really far
            1600, // 2 too close
            1710, // 3 near
            1800, // 4 medium
            2015, // 5  far
            1600, // 6 too close
            1710, // 7 medium
            1800, // 8 medium
            2015, // 9  far
            1710, // 10 near
            1800, // 11 medium
            2015, // 12 really far
            2110}; // 13 really far - ZONES indicated zone by number


    @Override
    public void init() {
        //aprilTagWebCam.init(hardwareMap, telemetry);
        aprilTagWebcam = new AprilTagSubsystem(hardwareMap);
        indicatorLights = new IndicatorLights(hardwareMap);


    }

    @Override
    public void loop() {

        if (!exposureSet) {
            //aprilTagWebCam.setManualExposure(10, 6);
            aprilTagWebcam.setManualExposure(telemetry, 10, 100, 4000);
            exposureSet = true;
        }
        // update the vision portal
        aprilTagWebcam.update();

        if (localizationUpdate()) {
            telemetryAprilTag();
            targetingUpdate();
        } else {
            indicatorLights.display(IndicatorLights.targeting.OFF, 99.9, 0.0); // should be actual ideal distance for shooting
        };



        telemetry.update();
    }

    private void telemetryAprilTag() {
        AprilTagDetection tag;
        if (aprilTagWebcam.tagDetectedWithId(20)) {
            //tag = aprilTagWebcam.tagWithId(20);
            tag = currentTagValues;
            telemetry.addLine(String.format("RobotPose: XYZ %6.1f %6.1f %6.1f  (inch)",
                    tag.robotPose.getPosition().x,
                    tag.robotPose.getPosition().y,
                    tag.robotPose.getPosition().z));
            telemetry.addLine(String.format("FTC: XYZ Range Bearing %6.1f %6.1f %6.1f %6.1f %6.1f (inch)",
                    tag.ftcPose.x,
                    tag.ftcPose.y,
                    tag.ftcPose.z,
                    tag.ftcPose.range,
                    tag.ftcPose.bearing));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f %6.1f (deg)",
                    tag.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                    tag.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                    tag.robotPose.getOrientation().getYaw(AngleUnit.DEGREES),
                    tag.metadata.tagsize));
        }

    } // end telemetryAprilTag()

    public boolean localizationUpdate() {
        // at this point we want to check if we have a valid GOAL April Tag available
        // if YES, then we update the localizer based on the Kalman Filtered April Tag robot pose
        // keeping in mind that in the AprilTagDetection object, we are storing the robot pose in the FTCPose
        AprilTagDetection tag;
        Pose cameraPose;
        double x, y, heading;
        double idealShootingAngle;

        if (aprilTagWebcam.tagDetectedWithId(20)) {
            currentTagValues = aprilTagWebcam.tagWithId(20);
            // convert from FTC Decode RobotPose to PedroPathing
            // using the audience side perspective
            // FTC Decode RobotPose: positive y axis right, negative x axis forward (orients better from blue goal side)
            // PedroPathing: positive x axis right, positive y axis forward
            x = currentTagValues.ftcPose.y + 72;
            y = -currentTagValues.ftcPose.x + 72;

            return true;

        } // tag 20 visible BLUE Goal
        else {return false;}
    } // end localizationUpdate

    public void targetingUpdate() {
        // at this point we want to check if we have a valid GOAL April Tag available
        // if YES, then we update the localizer based on the Kalman Filtered April Tag robot pose
        // keeping in mind that in the AprilTagDetection object, we are storing the robot pose in the FTCPose

        double x, y, heading;
        double idealShootingAngle;
        int zone=0;
        // convert from FTC Decode RobotPose to PedroPathing
        // using the audience side perspective
        // FTC Decode RobotPose: positive y axis right, negative x axis forward (orients better from blue goal side)
        // PedroPathing: positive x axis right, positive y axis forward
        // THEORETICALLY, the current tag FTC pose should be the correct Pedro Robot Pose, since we have just run localizationupdate()
        x = currentTagValues.ftcPose.y + 72;
        y = -currentTagValues.ftcPose.x + 72;
        //x = currentTagValues.ftcPose.x;
        //y = currentTagValues.ftcPose.y;

        distanceAprilTag = currentTagValues.ftcPose.range;
        angleAprilTag = currentTagValues.ftcPose.bearing;

        zone = calculateShootingAngle(x, y);
        telemetry.addLine(String.format("For zone calculation: x, y %6.1f %6.1f",
                x,
                y));

        idealShootingAngle = shootingAngles[zone];

        shooterTargetVelocity = calculateShootingVelocity(distanceAprilTag);

        telemetry.addLine(String.format("Current Targeting: zone, range, angle, ideal angle, RPM %d %6.1f %6.1f %6.1f %6.1f",
                zone,
                distanceAprilTag,
                angleAprilTag,
                idealShootingAngle,
                shooterTargetVelocity));


        indicatorLights.display(IndicatorLights.targeting.GREEN, angleAprilTag, idealShootingAngle); // should be actual ideal distance for shooting

    } // end targetingUpdate()

    public int calculateShootingAngle(double x, double y) {

        int zone;

        if (y<48) {
            zone = 1;
        } else {
            if (y>120) { // TOP row
                if (x>96) { // zone 5
                    zone = 5;
                } else if (x>72) { // zone 4
                    zone = 4;
                } else if (x>48) { // zone 3
                    zone = 3;
                } else { // zone 2
                    zone = 2;
                } // end of y > 120 top row

            } else if (y>96) { // second from TOP row
                if (x>96) { // zone 9
                    zone = 9;
                } else if (x>72) { // zone 8
                    zone = 8;
                } else if (x>48) { // zone 7
                    zone = 7;
                } else { // zone 6
                    zone = 6;
                } // end of y > 196 second from top row

            } else { // bottom of big launch zone
                if (x>96) { // zone 13
                    zone = 13;
                } else if (x>72) { // zone 12
                    zone = 12;
                } else if (x>48) { // zone 11
                    zone = 11;
                } else { // zone 10
                    zone = 10;
                } // end of y > 72 bottom row
            }
        }

        return zone;
    } // end of calculateShootingAngle()

    private double calculateShootingVelocity(double range) {
        double[] TARGETVELOCITY = {2110, // 0 really far
                2110, // 1 really far
                1600, // 2 too close
                1710, // 3 near
                1800, // 4 medium
                2015, // 5  far
                1600, // 6 too close
                1710, // 7 medium
                1800, // 8 medium
                2015, // 9  far
                1710, // 10 near
                1800, // 11 medium
                2015, // 12 really far
                2110}; // 13 really far - ZONES indicated zone by number

        double rpm = 2000;

        if (range>120) {
            rpm = 2210;
        } else if (range>100) {
            rpm = 2100;
        } else if (range> 80) {
            rpm = 2000;
        } else if (range>60) {
            rpm = 1900;
        } else if (range>40) {
            rpm = 1800;
        } else {
            rpm = 1600;
        }

        return rpm;
    } // end of calculateShootingVelocity()

} // end class TeleOpByTylerNov26


