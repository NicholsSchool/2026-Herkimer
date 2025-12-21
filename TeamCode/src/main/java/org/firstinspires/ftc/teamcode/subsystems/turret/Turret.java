package org.firstinspires.ftc.teamcode.subsystems.turret;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.LightManager;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class Turret extends SubsystemBase implements TurretConstants {

    private TurretIO io;
    private final TurretIO.TurretIOInputs inputs = new TurretIO.TurretIOInputs();
    public PIDController turretPIDController = new PIDController(2, 0.0, 0.0);    // for goToPosition: 0.75, 0,, 0
    public boolean aimTagDetected = false;
    public double aimError = 0.0, aimTagDistance = 0.0;

    public Turret(TurretIO io) {this.io = io;}

    public double desiredVelocity = 0.0;


    @Override
    public void periodic() {

        io.updateInputs(inputs);

        if (!PoseEstimator.getATResults().isPresent()) {
            LightManager.setTopLight(0);
            aimTagDetected = false;
            return;
        }

        aimTagDetected = false;
        for(AprilTagDetection tag: PoseEstimator.getATResults().get()){
            if(tag.id == TAGID){
                aimTagDetected = true;
                LightManager.setTopLight(LightManager.LightConstants.Green);
                aimTagDistance = distanceFromTag(tag.ftcPose.range);
                aimError = ((tag.center.x - ((double)frameWidth / 2)) / (double)frameWidth / 2);
            }
        }
    }



    public void turretSetAngle(double angle){
//        turretSetPower((angle - inputs.turretAngle) * turretP);
    }
//     io.turretSetPower(turretPIDController.calculate(inputs.turretAngle, angle));

    public void turretSetPower(double power){
        io.turretSetPower(power);
    }

    public double getTurretPosition(){
        return inputs.turretAngle;
    }

    /* Angle relative to protractor */
    public void redirectorSetAngle(AngleUnit unit, double angle){
        double angleRadians = unit.toRadians(angle);
        io.redirectorSetPosition(-1.27324 * (angleRadians + 0.12217304764) + 0.9);
    }

    public void redirectorAimAtDistance() {
        if (aimTagDistance > 2.5) {
            redirectorSetAngle(AngleUnit.RADIANS, 0.5708);
        } else {
            redirectorSetAngle(AngleUnit.RADIANS, 0.3491);
        }
    }

    public double distanceFromTag(double rawDistance){
            return ((rawDistance) - 1.02857/25.34286);
    }

    public AutoUtil.AutoActionState autoAim(){

        if (!aimTagDetected) {
            turretSetPower(0);
            return AutoUtil.AutoActionState.IDLE;
        }

        redirectorAimAtDistance();

        if (Math.abs(aimError) < 0.05) {
            turretSetPower(0);
            return AutoUtil.AutoActionState.FINISHED;
        }

        turretSetPower(-turretPIDController.calculate(aimError));

        return AutoUtil.AutoActionState.RUNNING;

    }

    public double getShooterVelocity(){
        return inputs.shooterVelocity;
    }


    //IN M/S
    public void setShooterVelocity(double velocity){
        io.shooterSetVelocity(234.25 * velocity);
    }


    public void runShooterForDistance() {
        desiredVelocity = 0.69 * (aimTagDistance - 1) + 5.17699;
        setShooterVelocity(desiredVelocity);
    }
}
