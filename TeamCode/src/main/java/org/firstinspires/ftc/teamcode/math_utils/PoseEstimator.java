package org.firstinspires.ftc.teamcode.math_utils;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainConstants;

import java.util.Optional;

/**
 * The Robot Pose (x, y, theta)
 */
public class PoseEstimator implements DrivetrainConstants {
    public Pose2D initialPose;
    public Pose2D robotPose;

    public GoBildaPinpointDriver pinpoint;
    public DcMotor odomX, odomY;

    public boolean useLL;
    public boolean isUsingLL;

    /**
     * The Field-Relative Robot Pose.
     * @param hwMap OpMode Hardware Map passthrough for LL, OTOS, and Gyro initialization.
     * @param initialPose Pose2D for robot's initial field-relative position.
     */
    public PoseEstimator(HardwareMap hwMap, Pose2D initialPose, boolean useLL) {
        this.useLL = useLL;
        Optional<Point> LLPose = Optional.empty();
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setOffsets(-4, -17, DistanceUnit.CM);
        pinpoint.initialize();

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setPosition(initialPose);
        pinpoint.update();

//        if (useLL) {
//            limelight = new LimelightComponent(hwMap, otos::getYawRate);
//            limelight.updateWithPose(initialPose.getHeading(AngleUnit.DEGREES));
//            LLPose = limelight.getRobotPose();
//        }

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

    public Pose2D getPose() { return robotPose; }

    public boolean isUsingLL() { return isUsingLL; }

    public void update() {
        pinpoint.update();

        robotPose = pinpoint.getPosition();
    }

    public double getInitialHeading(AngleUnit unit) {
        return initialPose.getHeading(unit);
    }

    private double getFieldHeading(AngleUnit unit) {
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
    private Vector transformFieldOriented(Vector inputVector) {
        return new Vector(
                (Math.cos(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.x) - (Math.sin(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.y),
                (Math.sin(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.x) + (Math.cos(initialPose.getHeading(AngleUnit.RADIANS)) * inputVector.y)
        );
    }

    /**
     * Method for testing the robot to field transformation using the OTOS data (not deltas)
     * @return The transformed Vector.
     */
    public Pose2D debugTransform() {
        Vector pinpointPos = new Vector(pinpoint.getPosX(DistanceUnit.METER), pinpoint.getPosY(DistanceUnit.METER));
        Vector transformedPos = transformFieldOriented(pinpointPos);

        return new Pose2D(DistanceUnit.METER, initialPose.getX(DistanceUnit.METER) + transformedPos.x, initialPose.getY(DistanceUnit.METER) + transformedPos.y, AngleUnit.DEGREES, getFieldHeading(AngleUnit.DEGREES));
    }

    public void resetOTOSIMU(){
        pinpoint.resetPosAndIMU();
    }

}