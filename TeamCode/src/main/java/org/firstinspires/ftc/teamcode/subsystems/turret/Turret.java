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
    public PIDController turretPIDController = new PIDController(kTP, kTI, kTD);
    public Vector aimDiffVector = new Vector(0.0, 0.0);
    public Vector aimDiffVectorGhost = new Vector(0.0, 0.0);

    public Turret(TurretIO io) {
        this.io = io;
        turretSetAngle(0.0, AngleUnit.DEGREES, 0.0);
    }

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

    /**
     * Sets the AprilTag ID for which Tag we are tracking
     * @param id the AprilTag ID
     * */
    public void setTagID(int id) {
        tagID = id;
        if (id == redTagID) {
            inputs.aprilTagPos = TurretConstants.redTagPos;
        } else if (id == blueTagID) {
            inputs.aprilTagPos = TurretConstants.blueTagPos;
        }
    }

    /**
     * AutoAim without velocity prediction and Auto Util states
     * */

    public void turretAutoAim(){

        turretSetPoint = Angles.clipRadians(aimDiffVector.angle() - PoseEstimator.getPose().getHeading(AngleUnit.RADIANS) + Math.toRadians(180));

        turretPIDPower = (Math.abs(getTurretPosition(AngleUnit.RADIANS) - (turretSetPoint)) < AngleUnit.RADIANS.fromDegrees(turretAimThresholdDegrees)) ? 0 :
                -turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS));

        turretSetPower(turretPIDPower + (TurretConstants.turretFeedForward * Math.signum(turretPIDPower)));
    }

    /**
     * @return a boolean for if the Turret is at the current SetPoint
     * */

    public boolean turretAtGoal(){
        return Math.abs(getTurretPosition(AngleUnit.RADIANS) - (turretSetPoint)) < AngleUnit.RADIANS.fromDegrees(2);
    }

    /**
     * A go to position for the Turret Angle
     * @param angle The angle to set the turret to
     * @param unit The unit for the given angle
     * @param turretManualOffset a variable to feed through the manual offset from TeleOp
     * */

    public void turretSetAngle(double angle, AngleUnit unit, double turretManualOffset) {
        turretSetPoint = unit.toRadians(angle)  + Math.toRadians(turretManualOffset);
        turretPIDController.setSetpoint(turretSetPoint);
        turretPIDController.reset();
    }

    /**
     * provides raw power to the Turret Servos
     * @param power the power supplied to the Turret Servos
     * */

    public void turretSetPower(double power) {
        io.turretSetPower(power);
    }

    /**
     * Gets the current Turret Position
     * @param angleUnit the Unit the angle will return in
     * @return the current Turret position as a double
     * */

    public double getTurretPosition(AngleUnit angleUnit) {
        if (angleUnit == AngleUnit.RADIANS) {
            return inputs.turretAngle;
        } else {
            return Math.toDegrees(inputs.turretAngle);
        }
    }


    /**
     * Automatically aims the Turret towards the goal, accounting for velocity and predicted rotational translation
     * @param turretManualOffset a variable to feed through the manual offset from TeleOp
     * */
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

    /**
     * @return the power being given to the Turret servos
     * */

    public double getTurretPower(){
        return inputs.turretPower;
    }

    /**
     * Gets the distance the Robot is away from the Goal
     * @param distanceUnit The distance Unit the distance will be returned in (ONLY INCH OR METER)
     * @return the distance the Robot is from the Goal
     * */

    public double getGoalDistance(DistanceUnit distanceUnit) {
        if (distanceUnit == DistanceUnit.INCH) {
            return aimDiffVector.magnitude();
        } else {
            return (aimDiffVector.magnitude() / 39.37);
        }
    }

    /**
     * @return The predicted position that the Robot will be at at any given time based off of the current velocity of the robot
     * */

    public Pose2D predictedPosition(){
        return new Pose2D (
                DistanceUnit.METER,
                PoseEstimator.getPose().getX(DistanceUnit.METER) + PoseEstimator.getRobotVelocityX(),
                PoseEstimator.getPose().getY(DistanceUnit.METER) + PoseEstimator.getRobotVelocityY(),
                AngleUnit.RADIANS,
                PoseEstimator.getPose().getHeading(AngleUnit.RADIANS) + PoseEstimator.getRobotVelocityHeading());
    }

    /**
     * @return The change in angle (delta theta) between the aim of the predicted velocity ghost and the actual robot
     * */

    public double getDeltaTheta () {
        return aimDiffVectorGhost.angle() - aimDiffVector.angle();
    }

    /**
     * @return The current angle* the Hood is at *likely not in degrees
     * */

    public double getHoodAngle() {
        return inputs.hoodAngle;
    }

    /**
     * Sets the angle of the Hood based on Servo position
     * @param position The position to set the Hood to
     * */

    public void hoodSetServoPosition(double position) {
        io.hoodSetPosition(position);
    }

    /**
     * Moves the Mechanical Stop in to block Artifact traffic
     * */

    public void moveStopIn() {
        io.setMechStopPosition(mechanicalStopIn);
    }

    /**
     * takes the Mechanical Stop out to resume Artifact traffic
     * */

    public void takeStopOut(){
        io.setMechStopPosition(mechanicalStopOut);
    }

    public boolean flywheelAtGoal(){
        return Math.abs(getShooterVelocity() - getAcceleratorSetpoint()) < SHOOT_SPEED_TOLERANCE;
    }

    /**
     * @return True if the Robot is within the preferred shooting range
     * */

    public boolean inShootingRange(){
        return getGoalDistance(DistanceUnit.METER) > shootingMinRange && getGoalDistance(DistanceUnit.METER) < shootingMaxRange;
    }

    /**
     * @return The distance from the Robot to the AprilTag without accounting for the Turret not being in the center
     * */

    public double distanceFromTag(double rawDistance) {
        return ((rawDistance) - 1.02857 / 25.34286);
    }

    /**
     * Autonomous Turret aiming without velocity prediction
     * @return An Auto Util ActionState that tells the Auto if the action is finished
     * */

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

    /**
     * Gets the current Setpoint (desired position) of the Turret
     * @param unit The unit the Setpoint will be returned in
     * @return The current Turret Setpoint
     * */

    public double getTurretSetpoint(AngleUnit unit) {
        return unit.fromRadians(turretSetPoint);
    }

    /**
     * Gets the current velocity the Shooter wheel is spinning at
     * @return The current velocity of the Shooter in Ticks
     * */

    public double getShooterVelocity() {
        return inputs.shooterVelocity;
    }

    /**
     * Gets the raw angle of the Turret (without the Ticks to Degrees regression) for tuning
     * @return The raw angle of the Turret
     * */

    public double getRawTurretPos() {
        return inputs.rawTurretAngle;
    }

    /**
     * Gets the aim error between the current Turret position and its desired position
     * @param unit The unit the error will be returned in
     * @return The aim error between the Turret and its Setpoint (desired position)
     * */

    public double getAimError(AngleUnit unit) {
        return (Math.abs(getTurretPosition(unit) - (unit.fromRadians(turretSetPoint))));
    }

    /**
     * Sets the Shooter velocity in METERS PER SECOND which we DON'T USE RIGHT NOW
     * @param velocity The velocity IN M/S
     * */

    public void setShooterVelocity(double velocity) {
        io.shooterSetVelocity(234.25 * velocity);
    }

    /**
     * Sets the Shooter velocity in TICKS which we DO USE
     * @param velocity The velocity in Ticks
     * */

    public void setShooterVelocityTicks(double velocity) {
        io.shooterSetVelocity(velocity);
    }

    /**
     * Gets the Setpoint of the Shooter wheel
     * @return the current desired velocity of the flywheel
     * */

    public double getAcceleratorSetpoint() {
        return acceleratorSetpoint;
    }

    /**
     * Automatically sets the velocity of the Shooter wheel and the Servo position of the Hood based off of the distance the robot is from the goal based on a regression
     * */

    public void autoAccelerate() {
        setShooterVelocityTicks(acceleratorSetpoint);
        //tape
        hoodSetServoPosition(-0.0720368 * (Math.pow(getGoalDistance(DistanceUnit.METER), 2)) + (0.344157 * getGoalDistance(DistanceUnit.METER)) + 0.0680431);
        //no tape
        //hoodSetServoPosition((0.105132 * (Math.pow(getGoalDistance(DistanceUnit.METER), 2))) - (0.535538 * getGoalDistance(DistanceUnit.METER)) + 0.98676);

    }

    /**
     * Resets the Turret encoder
     * */
    public void resetTurretEncoder(){
        io.resetTurretEncoder(inputs);
    }




}
