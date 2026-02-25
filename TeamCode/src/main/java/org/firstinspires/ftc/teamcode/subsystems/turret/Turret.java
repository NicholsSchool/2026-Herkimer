package org.firstinspires.ftc.teamcode.subsystems.turret;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
//import org.firstinspires.ftc.teamcode.subsystems.LightManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

@Config
public class Turret extends SubsystemBase implements TurretConstants {

    private TurretIO io;
    private final TurretIO.TurretIOInputs inputs = new TurretIO.TurretIOInputs();
    public static double kTP = 0.7, kTI = 0.015, kTD = 0.08;
    public PIDController turretPIDController = new PIDController(kTP, kTI, kTD);
    public boolean aimTagDetected = false;
    public Vector aimDiffVector = new Vector(0.0, 0.0);
    public static double acceleratorSetpoint = 1200; //make static for tuning
    public static double redirectorSetpoint = 0.0;
//    public PIDController velocityPIDController = new PIDController(4,0.0,0.05);

    public Turret(TurretIO io) {
        this.io = io;
        //turretPIDController.setIZone(AngleUnit.RADIANS.fromDegrees(12.0)); //Only uses I when error < 5deg
    }

    public double desiredVelocity = 0.0;

    public double tagID = DEFAULT_TAGID;

    public double setPoint;

    @Override
    public void periodic() {

        io.updateInputs(inputs);

        //aimTagDistance = Math.hypot((PoseEstimator.getPose().getX(DistanceUnit.INCH) - inputs.aprilTagPos.getX(DistanceUnit.INCH)), (PoseEstimator.getPose().getY(DistanceUnit.INCH) - inputs.aprilTagPos.getY(DistanceUnit.INCH)));
        //eventually i wanna use the distance from the center of the robot to the center of the goal rather than the aprilTag, but we would have to redo the regressions

        Pose2D turretCenter = new Pose2D(DistanceUnit.CM,
                PoseEstimator.getPose().getX(DistanceUnit.CM) - (7.95 * Math.cos(PoseEstimator.getPose().getHeading(AngleUnit.RADIANS))),
                PoseEstimator.getPose().getY(DistanceUnit.CM) - (7.95 * Math.sin(PoseEstimator.getPose().getHeading(AngleUnit.RADIANS))),
                AngleUnit.RADIANS,
                PoseEstimator.getPose().getHeading(AngleUnit.RADIANS)
        );

        aimDiffVector = new Vector((turretCenter.getX(DistanceUnit.INCH) - inputs.aprilTagPos.getX(DistanceUnit.INCH)),
                (turretCenter.getY(DistanceUnit.INCH) - inputs.aprilTagPos.getY(DistanceUnit.INCH)));

    }

    public void setTagID(int id) {
        tagID = id;
        if (id == 24) {
            inputs.aprilTagPos = TurretConstants.redTagPos;
        } else if (id == 20) {
            inputs.aprilTagPos = TurretConstants.blueTagPos;
        }
    }

    public void turretSetAngle(double angle, AngleUnit unit) {
//        turretSetPower((angle - inputs.turretAngle) * turretP);
        setPoint = unit.toRadians(angle);
        if (setPoint < -Math.PI / 2 || setPoint > Math.PI / 2) {
//            turretSetPower(0);
        } else {
            turretPIDController.setSetpoint(setPoint);
            if(-turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS)) < 0.1 && (-turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS)) > 0.01)) {
                turretSetPower(Range.clip(-turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS) - 0.05), -1, 1));
            }else if(-turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS)) > -0.1 && (-turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS)) < -0.01)) {
                turretSetPower(Range.clip(-turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS) + 0.05), -1, 1));
            }else{
                turretSetPower(Range.clip(-turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS)), -1, 1));
            }
        }
    }
//     io.turretSetPower(turretPIDController.calculate(inputs.turretAngle, angle));

    public void turretSetPower(double power) {
        io.turretSetPower(power);
    }

    public double getTurretPosition(AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.RADIANS) {
            return inputs.turretAngle;
        } else {
            return Math.toDegrees(inputs.turretAngle);
        }
    }

    public double getTurretPIDPower(){
        return (Range.clip(-turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS)), -1, 1));
    }


    public double getGoalDistance(DistanceUnit distanceUnit) {
        if (distanceUnit == DistanceUnit.INCH) {
            return aimDiffVector.magnitude();
        } else {
            return (aimDiffVector.magnitude() / 39.37);
        }
    }

    public double getRedirectorPower() {
        return inputs.redirectorPower;
    }

    public void redirectorSetVelocity(double velocity) {
        io.redirectorSetVelocity(velocity);
    }

    public double getRedirectorVelocity() {
        return inputs.redirectorVelocity;
    }


    public double distanceFromTag(double rawDistance) {
        return ((rawDistance) - 1.02857 / 25.34286);
    }

    public AutoUtil.AutoActionState autoAim() {


        setPoint = Angles.clipRadians(aimDiffVector.angle() - PoseEstimator.getPose().getHeading(AngleUnit.RADIANS) + Math.toRadians(180));
        if (Math.abs(getTurretPosition(AngleUnit.RADIANS) - (setPoint)) < AngleUnit.RADIANS.fromDegrees(2)) {
            turretSetPower(0);
            return AutoUtil.AutoActionState.FINISHED;
        }

        //(Math.abs(getTurretPosition(AngleUnit.RADIANS) - (setPoint))

        if (setPoint < -Math.PI / 2 || setPoint > Math.PI / 2) {
//            turretSetPower(0);
        } else {
            turretSetAngle(setPoint, AngleUnit.RADIANS);
        }


        return AutoUtil.AutoActionState.RUNNING;

    }

    public double getTurretSetpoint(AngleUnit unit) {
        return unit.fromRadians(setPoint);
    }

    public double getShooterVelocity() {
        return inputs.shooterVelocity;
    }

    public double getAcceleratorSetpoint() {
        return acceleratorSetpoint;
    }

    public double getRawTurretPos() {
        return inputs.rawTurretAngle;
    }

//    public double getTurretSetpoint(){
//        return Angles.clipDegrees(Math.toDegrees(aimTagDistance.angle()) - PoseEstimator.getPose().getHeading(AngleUnit.DEGREES));
//    }

    public double getAimError(AngleUnit unit) {
        return (Math.abs(getTurretPosition(unit) - (unit.fromRadians(setPoint))));
    }


    //IN M/S
    public void setShooterVelocity(double velocity) {
        io.shooterSetVelocity(234.25 * velocity);
    }

    public void setShooterVelocityTicks(double velocity) {
        io.shooterSetVelocity(velocity);
    }


    public void runShooterForDistance() {
        desiredVelocity = 0.69 * (aimDiffVector.magnitude()) + 5.17699;
        setShooterVelocity(desiredVelocity);
    }

    public void autoAccelerate() {
        setShooterVelocityTicks(acceleratorSetpoint);
        //the quadratic redirector function
//        redirectorSetVelocity((-27.6) * Math.pow(getGoalDistance(DistanceUnit.METER), 2) + (-117.4 * (getGoalDistance(DistanceUnit.METER))) + 14.95);
        //NEW quadratic redirector function
        redirectorSetVelocity((-20.31152)* Math.pow(getGoalDistance(DistanceUnit.METER), 2) + (-101.9002 * (getGoalDistance(DistanceUnit.METER))) + 18.06928);
//        redirectorSetVelocity(redirectorSetpoint);
    }

    //the redirector power function
    //redirectorSetVelocity((-122.5) * (Math.pow(getGoalDistance(DistanceUnit.METER), 1.4315)));

    //the redirector linear function
    //redirectorSetVelocity((-258 * getGoalDistance) + 169);


    public double getRedirectorSetpoint() {
        return (-314.28571 * getGoalDistance(DistanceUnit.METER) + 290.47619);
    }

    public void resetTurretEncoder(){
        io.resetTurretEncoder(inputs);
    }

}
