package org.firstinspires.ftc.teamcode.math_utils;

import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
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

    /**
     * The Field-Relative Robot Pose.
     * @param hwMap OpMode Hardware Map passthrough for LL, OTOS, and Gyro initialization.
     * @param initialPose Pose2D for robot's initial field-relative position.
     * @param useAT True if using a Camera/AprilTag system.
     * @param forceReset resets the Pose to 0,0 before TeleOp. Turn true for testing just TeleOp, but do NOT EVER GO TO COMPETITION USING IT AS TRUE PLEASE MAKE IT FALSE
     */
    public static void init(HardwareMap hwMap, Pose2D initialPose, boolean useAT, boolean forceReset) {

        PoseEstimator.initialPose = initialPose;
        PoseEstimator.robotPose = initialPose;
        PoseEstimator.useAT = useAT;
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-1.9, -15.6, DistanceUnit.CM);
        if (forceReset) pinpoint.setPosition(initialPose);
        pinpoint.initialize();
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.update();

        pinpoint.recalibrateIMU();
        pinpoint.update();

//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        //builder.setCamera(hwMap.get(WebcamName.class, "W"));
//        builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
//        builder.setCameraResolution(new Size(1280, 720));
//        builder.addProcessor(aprilTag);
//        VisionPortal visionPortal = builder.build();
//        visionPortal.resumeStreaming();


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

    /**
     * resets the Heading of the Robot
     * */

    public static void resetIMU(){
        pinpoint.setPosition( new Pose2D(DistanceUnit.INCH, getPose().getX(DistanceUnit.INCH), getPose().getY(DistanceUnit.INCH), AngleUnit.DEGREES, 0));
        pinpoint.update();
    }

    /**
     * resets the Pose of the Robot to the initial Pose of the Auto.
     * */

    public static void resetPoseToAutoStart(boolean isRed){
        pinpoint.setPosition(allianceFlip(isRed, new Pose2D(DistanceUnit.METER, -1.6, -1, AngleUnit.DEGREES, 0)));
        pinpoint.update();
    }

    /**
     * Sets the Pose of the Robot to a different one (it sets it where it is, it does not move to this position).
     * @param inputPose the Pose to set the current Pose to.
     * */

    public static void setPosition(Pose2D inputPose){
        pinpoint.setPosition(inputPose);
        pinpoint.update();
    }

    /**
     * Gets the current Position of the Robot.
     * @return A Pose2D of the current Position of the Robot.
     * */

    public static Pose2D getPose() { return robotPose; }

    /**
     * Gets the current velocity of the Robot in the X direction.
     * @return The current velocity of the Robot in the X direction.
     * */

    public static double getRobotVelocityX(){
        return pinpoint.getVelX(DistanceUnit.METER);
    }

    /**
     * Gets the current velocity of the Robot in the Y direction.
     * @return The current velocity of the Robot in the Y direction.
     * */

    public static double getRobotVelocityY(){
        return pinpoint.getVelY(DistanceUnit.METER);
    }

    /**
     * Gets the current velocity of the Robot Heading.
     * @return The current velocity of the Robot Heading in RADIANS.
     * */

    public static double getRobotVelocityHeading(){
        return pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
    }

    /**
     * Gets a Pose2D of all of the current velocities of the robot to predict the next position of the Robot.
     * @return A Pose2D of all of the current velocities of the robot.
     * */

    public static Pose2D getRobotVelocity(){
        return new Pose2D(DistanceUnit.METER, getRobotVelocityX(), getRobotVelocityY(), AngleUnit.DEGREES, getRobotVelocityHeading());
    }

    /**
     * Updates the position of the Robot.
     * */

    public static void periodic() {
        pinpoint.update();

        robotPose = pinpoint.getPosition();

//        latestATResults = Optional.ofNullable(aprilTag.getDetections());
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

    /**
     * Gets the current status of the Pinpoint.
     * @return The current status of the Pinpoint.
     * */
    public static GoBildaPinpointDriver.DeviceStatus getPinpointStatus() { return pinpoint.getDeviceStatus(); }


    /**
     * Flips a Pose to be on the other side of the field.
     * @return A new Pose2D with the Y direction and angles flipped to account for being on the other side of the field
     * */
    public static Pose2D allianceFlip(boolean red, Pose2D inputPose) {
        return new Pose2D(
                DistanceUnit.INCH,
                inputPose.getX(DistanceUnit.INCH),
                inputPose.getY(DistanceUnit.INCH) * (red ? -1 : 1),
                AngleUnit.DEGREES,
                Angles.clipDegrees(inputPose.getHeading(AngleUnit.DEGREES) * (red ? -1 : 1))
        );
    }

}