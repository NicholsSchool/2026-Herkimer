package org.firstinspires.ftc.teamcode.subsystems.turret;


import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

public class Turret extends SubsystemBase implements TurretConstants {

    private TurretIO io;
    private final TurretIO.TurretIOInputs inputs = new TurretIO.TurretIOInputs();
    public PIDController turretPIDController;

    public Turret(TurretIO io) {
        this.io = io;
        turretPIDController = new PIDController(turretP, turretI, turretD);
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void turretSetAngle(double angle){
        turretSetPower((angle - inputs.turretAngle) * turretP);
    }
//     io.turretSetPower(turretPIDController.calculate(inputs.turretAngle, angle));

    public void turretSetPower(double power){
        io.turretSetPower(power);
    }

    public double getTurretPosition(){
        return inputs.turretAngle;
    }



    public void shooterSetVelocity(double velocity){
        io.shooterSetVelocity(velocity);
    }

    //shooter angle so 90 + the protractor line
    //ITS IN RADIANS
    public void redirectorSetAngle(double angle){
        io.redirectorSetPosition(-1.27324 * (angle + 0.12217304764) + 0.9);
    }


    public void redirectorSetPos(double pos){
        io.redirectorSetPosition(pos);
    }

    public double getRawTurretPos(){
        return inputs.rawTurretAngle;
    }

    public double getShooterVelocity(){
        return inputs.shooterVelocity;
    }

    //IN M/S
    public void setActualVelocity(double velocity){
        io.shooterSetVelocity(234.25 * velocity);
    }



}
