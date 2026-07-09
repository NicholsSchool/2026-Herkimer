package org.firstinspires.ftc.teamcode.subsystems.turret;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.RobotAutoDriveByEncoder_Linear;
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
    public static double kTP = 0.1, kTI = 0.0, kTD = 0.0;
    public PIDController turretPIDController = new PIDController(kTP, kTI, kTD);
    public boolean aimTagDetected = false;
    public Vector aimDiffVector = new Vector(0.0, 0.0);
    public Vector aimDiffVectorGhost = new Vector(0.0, 0.0);
    public static double acceleratorSetpoint = 2200; //make static for tuning
    public static double hoodPosition;
    public double rotationalPrediction = 0.29;
    public double rotationTranslationPrediction = -0.25;


    public Turret(TurretIO io) {
        this.io = io;
        turretSetAngle(0.0, AngleUnit.DEGREES, 0.0);
    }

    public double desiredVelocity = 0.0;

    public double tagID = DEFAULT_TAGID;

    public double turretSetPoint;
    double turretPIDPower = 0.0;

    @Override
    public void periodic() {

        io.updateInputs(inputs);

        if(inShootingRange()){
            io.setLightPosition(greenLight);
        }else{
            io.setLightPosition(0.0);
        }

        Pose2D turretCenter = new Pose2D(DistanceUnit.CM,
                PoseEstimator.getPose().getX(DistanceUnit.CM) - (7.95 * Math.cos(PoseEstimator.getPose().getHeading(AngleUnit.RADIANS))),
                PoseEstimator.getPose().getY(DistanceUnit.CM) - (7.95 * Math.sin(PoseEstimator.getPose().getHeading(AngleUnit.RADIANS))),
                AngleUnit.RADIANS,
                PoseEstimator.getPose().getHeading(AngleUnit.RADIANS)
        );

        aimDiffVector = new Vector((turretCenter.getX(DistanceUnit.INCH) - inputs.aprilTagPos.getX(DistanceUnit.INCH)),
                (turretCenter.getY(DistanceUnit.INCH) - inputs.aprilTagPos.getY(DistanceUnit.INCH)));

        aimDiffVectorGhost = new Vector(((turretCenter.getX(DistanceUnit.INCH) + predictedPosition().getX(DistanceUnit.INCH))
                - inputs.aprilTagPos.getX(DistanceUnit.INCH)),
                (turretCenter.getY(DistanceUnit.INCH) + predictedPosition().getY(DistanceUnit.INCH) - inputs.aprilTagPos.getY(DistanceUnit.INCH)));

        turretPIDPower = (Math.abs(getTurretPosition(AngleUnit.RADIANS) - (turretSetPoint)) < AngleUnit.RADIANS.fromDegrees(2)) ? 0 :
                -turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS));

        turretSetPower(turretPIDPower + (TurretConstants.turretFeedForward * Math.signum(turretPIDPower)));

    }

    public void setTagID(int id) {
        tagID = id;
        if (id == 24) {
            inputs.aprilTagPos = TurretConstants.redTagPos;
        } else if (id == 20) {
            inputs.aprilTagPos = TurretConstants.blueTagPos;
        }
    }

    public void turretAutoAim(){

        turretSetPoint = Angles.clipRadians(aimDiffVector.angle() - PoseEstimator.getPose().getHeading(AngleUnit.RADIANS) + Math.toRadians(180));

        turretPIDPower = (Math.abs(getTurretPosition(AngleUnit.RADIANS) - (turretSetPoint)) < AngleUnit.RADIANS.fromDegrees(2)) ? 0 :
                -turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS));

        turretSetPower(turretPIDPower + (TurretConstants.turretFeedForward * Math.signum(turretPIDPower)));
    }

    public boolean turretAtGoal(){
        return Math.abs(getTurretPosition(AngleUnit.RADIANS) - (turretSetPoint)) < AngleUnit.RADIANS.fromDegrees(2);
    }

    public void turretSetAngle(double angle, AngleUnit unit, double turretManualOffset) {
        turretSetPoint = unit.toRadians(angle)  + Math.toRadians(turretManualOffset);
        turretPIDController.setSetpoint(turretSetPoint);
        turretPIDController.reset();
    }

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

    public void turretAutoAimShootOnTheMove(double turretManualOffset) {
        double angle = Angles.clipRadians(
                aimDiffVector.angle()
                        - PoseEstimator.getPose().getHeading(AngleUnit.RADIANS)
                        + Math.toRadians(180)
                        - (PoseEstimator.getRobotVelocityHeading()
                        * rotationalPrediction)
                        - getDeltaTheta() * rotationTranslationPrediction);

        double clippedAngle = Range.clip(angle, turretMin, turretMax);
        turretSetAngle(clippedAngle, AngleUnit.RADIANS, turretManualOffset);
    }

    public double getTurretPower(){
        return inputs.turretPower;
    }

    public double getGoalDistance(DistanceUnit distanceUnit) {
        if (distanceUnit == DistanceUnit.INCH) {
            return aimDiffVector.magnitude();
        } else {
            return (aimDiffVector.magnitude() / 39.37);
        }
    }

    public Pose2D predictedPosition(){
        return new Pose2D (
                DistanceUnit.METER,
                PoseEstimator.getPose().getX(DistanceUnit.METER) + PoseEstimator.getRobotVelocityX(),
                PoseEstimator.getPose().getY(DistanceUnit.METER) + PoseEstimator.getRobotVelocityY(),
                AngleUnit.RADIANS,
                PoseEstimator.getPose().getHeading(AngleUnit.RADIANS) + PoseEstimator.getRobotVelocityHeading());
    }

    public double getDeltaTheta () {
        return aimDiffVectorGhost.angle() - aimDiffVector.angle();
    }

    public double getHoodAngle() {
        return inputs.hoodAngle;
    }

    public void hoodSetServoPosition(double position) {
        io.hoodSetPosition(position);
    }

    public void hoodSetDashboardPosition(){
        io.hoodSetPosition(hoodPosition);
    }

    public void moveStopIn() {
        io.setMechStopPosition(0.8);
    }

    public void takeStopOut(){
        io.setMechStopPosition(1.0);
    }

    public boolean flywheelAtGoal(){
        return Math.abs(getShooterVelocity() - getAcceleratorSetpoint()) < SHOOT_SPEED_TOLERANCE;
    }

    public boolean inShootingRange(){
        return getGoalDistance(DistanceUnit.METER) > shootingMinRange && getGoalDistance(DistanceUnit.METER) < shootingMaxRange;
    }

    public double distanceFromTag(double rawDistance) {
        return ((rawDistance) - 1.02857 / 25.34286);
    }

    public AutoUtil.AutoActionState autoAim() {

        turretSetPoint = Angles.clipRadians(aimDiffVector.angle() - PoseEstimator.getPose().getHeading(AngleUnit.RADIANS) + Math.toRadians(180));
        if (Math.abs(getTurretPosition(AngleUnit.RADIANS) - (turretSetPoint)) < AngleUnit.RADIANS.fromDegrees(2)) {
            turretSetPower(0);
            return AutoUtil.AutoActionState.FINISHED;
        }

        if (turretSetPoint < -Math.PI / 2 || turretSetPoint > Math.PI / 2) {
//            turretSetPower(0);
        } else {
            turretSetAngle(turretSetPoint, AngleUnit.RADIANS, 0.0);
        }

        return AutoUtil.AutoActionState.RUNNING;

    }

    public double getTurretSetpoint(AngleUnit unit) {
        return unit.fromRadians(turretSetPoint);
    }

    public double getShooterVelocity() {
        return inputs.shooterVelocity;
    }

    public double getRawTurretPos() {
        return inputs.rawTurretAngle;
    }

    public double getAimError(AngleUnit unit) {
        return (Math.abs(getTurretPosition(unit) - (unit.fromRadians(turretSetPoint))));
    }

    //IN M/S
    public void setShooterVelocity(double velocity) {
        io.shooterSetVelocity(234.25 * velocity);
    }

    public void setShooterVelocityTicks(double velocity) {
        io.shooterSetVelocity(velocity);
    }

    public double getAcceleratorSetpoint() {
        return acceleratorSetpoint;
    }

    public void runShooterForDistance() {
        desiredVelocity = 0.69 * (aimDiffVector.magnitude()) + 5.17699;
        setShooterVelocity(desiredVelocity);
    }

    public void autoAccelerate() {
        setShooterVelocityTicks(acceleratorSetpoint);
        //tape
        hoodSetServoPosition(-0.0720368 * (Math.pow(getGoalDistance(DistanceUnit.METER), 2)) + (0.344157 * getGoalDistance(DistanceUnit.METER)) + 0.0680431);
        //no tape
        //hoodSetServoPosition((0.105132 * (Math.pow(getGoalDistance(DistanceUnit.METER), 2))) - (0.535538 * getGoalDistance(DistanceUnit.METER)) + 0.98676);

    }

    public void resetTurretEncoder(){
        io.resetTurretEncoder(inputs);
    }




}
