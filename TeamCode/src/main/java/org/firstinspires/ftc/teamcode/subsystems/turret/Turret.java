package org.firstinspires.ftc.teamcode.subsystems.turret;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.LightManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class Turret extends SubsystemBase implements TurretConstants {

    private TurretIO io;
    private final TurretIO.TurretIOInputs inputs = new TurretIO.TurretIOInputs();
    public PIDController turretPIDController = new PIDController(0.8, 0.0, 0.0);
    public boolean aimTagDetected = false;
    public Vector aimTagDistance = new Vector(0.0, 0.0);

    public Turret(TurretIO io) {this.io = io;}

    public double desiredVelocity = 0.0;

    public double tagID = DEFAULT_TAGID;

    @Override
    public void periodic() {

        io.updateInputs(inputs);

        //aimTagDistance = Math.hypot((PoseEstimator.getPose().getX(DistanceUnit.INCH) - inputs.aprilTagPos.getX(DistanceUnit.INCH)), (PoseEstimator.getPose().getY(DistanceUnit.INCH) - inputs.aprilTagPos.getY(DistanceUnit.INCH)));
        //eventually i wanna use the distance from the center of the robot to the center of the goal rather than the aprilTag, but we would have to redo the regressions

        aimTagDistance = new Vector((PoseEstimator.getPose().getX(DistanceUnit.INCH) - inputs.aprilTagPos.getX(DistanceUnit.INCH)), (PoseEstimator.getPose().getY(DistanceUnit.INCH) - inputs.aprilTagPos.getY(DistanceUnit.INCH)));

    }

    public void setTagID(int id) {
        tagID = id;
    }

    public void turretSetAngle(double angle){
//        turretSetPower((angle - inputs.turretAngle) * turretP);
    }
//     io.turretSetPower(turretPIDController.calculate(inputs.turretAngle, angle));

    public void turretSetPower(double power){
        io.turretSetPower(power);
    }

    public double getTurretPosition(AngleUnit angleUnit){
        if (angleUnit == AngleUnit.RADIANS){
            return inputs.turretAngle;
        }else {
            return Math.toDegrees(inputs.turretAngle);
        }
    }

    /* Angle relative to protractor */
    public void redirectorSetAngle(AngleUnit unit, double angle){
        double angleRadians = unit.toRadians(angle);
        io.redirectorSetPosition(-1.27324 * (angleRadians + 0.12217304764) + 0.9);
    }

    public void redirectorAimAtDistance() {
        if (aimTagDistance.magnitude() > 2.5) {
            redirectorSetAngle(AngleUnit.RADIANS, 0.5708);
        } else {
            redirectorSetAngle(AngleUnit.RADIANS, 0.3491);
        }
    }

    public double distanceFromTag(double rawDistance){
            return ((rawDistance) - 1.02857/25.34286);
    }

    public AutoUtil.AutoActionState autoAim(){

        redirectorAimAtDistance();

        double setPoint = Angles.clipRadians(aimTagDistance.angle() - PoseEstimator.getPose().getHeading(AngleUnit.RADIANS));
        if (Math.abs(getTurretPosition(AngleUnit.RADIANS) - (setPoint)) < 0.065) {
            turretSetPower(0);
            return AutoUtil.AutoActionState.FINISHED;
        }

        if (setPoint < -Math.PI/2 || setPoint > Math.PI/2){
            turretSetPower(0);
        }else{
            turretPIDController.setSetpoint(setPoint);
            turretSetPower(turretPIDController.calculate(getTurretPosition(AngleUnit.RADIANS)));
//            turretSetPower((getTurretPosition(AngleUnit.DEGREES) - (setPoint)));
        }


     return AutoUtil.AutoActionState.RUNNING;

    }

    public double getShooterVelocity(){
        return inputs.shooterVelocity;
    }

    public double getShooterSetpoint() {
        return 23.425 * desiredVelocity;
    }

    public double getTurretSetpoint(){
        return Angles.clipDegrees(Math.toDegrees(aimTagDistance.angle()) - PoseEstimator.getPose().getHeading(AngleUnit.DEGREES));
    }

    public double getAimError(){
        return turretPIDController.getPositionError();
    }


    //IN M/S
    public void setShooterVelocity(double velocity){
        io.shooterSetVelocity(234.25 * velocity);
    }


    public void runShooterForDistance() {
        desiredVelocity = 0.69 * (aimTagDistance.magnitude()) + 5.17699;
        setShooterVelocity(desiredVelocity);
    }
}
