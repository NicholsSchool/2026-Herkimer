package org.firstinspires.ftc.teamcode.math_utils;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.BooleanSupplier;

/**
 * The Robot Pose (x, y, theta)
 */
public class PoseEstimator implements DrivetrainConstants {
    public static Pose2D initialPose;
    public static Pose2D robotPose;

    public static GoBildaPinpointDriver pinpoint;

    public static boolean useAT;
    public static AprilTagProcessor aprilTag;
    public static Optional<ArrayList<AprilTagDetection>> latestATResults = Optional.empty();

    /**
     * The Field-Relative Robot Pose.
     * @param hwMap OpMode Hardware Map passthrough for LL, OTOS, and Gyro initialization.
     * @param initialPose Pose2D for robot's initial field-relative position.
     */
    public static void init(HardwareMap hwMap, Pose2D initialPose, boolean useAT, boolean forceReset) {



        PoseEstimator.initialPose = initialPose;
        PoseEstimator.robotPose = initialPose;
        PoseEstimator.useAT = useAT;
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-4, -17, DistanceUnit.CM);
        if (forceReset || pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) pinpoint.setPosition(initialPose);
        pinpoint.initialize();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.update();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        builder.setCamera(hwMap.get(WebcamName.class, "W"));
        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setCameraResolution(new Size(1280, 720));
        builder.addProcessor(aprilTag);
        VisionPortal visionPortal = builder.build();
        visionPortal.resumeStreaming();


        //If the limelight can localize at startup, use that for the initial pose.
//        if (useLL && LLPose.isPresent() ) {
//            this.initialPose = new Pose2D(
//                    DistanceUnit.METER,
//                    LLPose.get().x,
//                    LLPose.get().y,
//                    AngleUnit.DEGREES,
//                    initialPose.getHeading(AngleUnit.DEGREES)
//            );
//        } else {
//            this.initialPose = new Pose2D(
//                    DistanceUnit.METER,
//                    initialPose.getX(DistanceUnit.METER),
//                    initialPose.getY(DistanceUnit.METER),
//                    AngleUnit.DEGREES,
//                    initialPose.getHeading(AngleUnit.DEGREES)
//            );
//
//            this.robotPose = this.initialPose;
//
//        }

    }

    public static void waitForPinpointInit(BooleanSupplier opModeIsActive) {
        while (opModeIsActive.getAsBoolean() || pinpoint.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {}
    }

    public static void resetIMU(){
        pinpoint.setPosition( new Pose2D(DistanceUnit.INCH, getPose().getX(DistanceUnit.INCH), getPose().getY(DistanceUnit.INCH), AngleUnit.DEGREES, 45));
        pinpoint.update();
    }

    public static void setPosition(Pose2D inputPose){
        pinpoint.setPosition(inputPose);
        pinpoint.update();
    }

    public static Pose2D getPose() { return robotPose; }

    public static void periodic() {
        pinpoint.update();

        robotPose = pinpoint.getPosition();

        latestATResults = Optional.ofNullable(aprilTag.getDetections());
    }

    private static double getFieldHeading(AngleUnit unit) {
        if (unit == AngleUnit.DEGREES) {
            return Angles.clipDegrees(initialPose.getHeading(AngleUnit.DEGREES) + pinpoint.getHeading(AngleUnit.DEGREES));
        } else {
            return Angles.clipRadians(initialPose.getHeading(AngleUnit.RADIANS) + pinpoint.getHeading(AngleUnit.RADIANS));
        }
    }

    /**
     * Takes in a vector that is robot-oriented (such as OTOS position/deltas) and converts it to
     * field-oriented using the field heading calculated from the gyro and inputted initialHeading.
     * @param inputVector The robot-oriented vector.
     * @return The field-oriented vector.
     */
    private static Vector transformFieldOriented(Vector inputVector) {
        return new Vector(
                (Math.cos(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.x) - (Math.sin(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.y),
                (Math.sin(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.x) + (Math.cos(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.y)
        );
    }

    public static Optional<ArrayList<AprilTagDetection>> getATResults() {
        return latestATResults;
    }

    public static GoBildaPinpointDriver.DeviceStatus getPinpointStatus() { return pinpoint.getDeviceStatus(); }

}