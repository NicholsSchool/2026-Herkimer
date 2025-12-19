package org.firstinspires.ftc.teamcode.subsystems.turret;

public interface TurretIO {

    public static class TurretIOInputs{
        public double turretAngle = Math.PI / 2;
        public double redirectorAngle  = 0.0;
        public double shooterVelocity = 0.0;
        public boolean magnetState = true;
        public double rawTurretAngle = 0.0;

    }

    public void updateInputs(TurretIOInputs inputs);
    public void shooterSetVelocity(double velocity);
    public void redirectorSetPosition(double angle);
    public void turretSetPower(double power);




}
