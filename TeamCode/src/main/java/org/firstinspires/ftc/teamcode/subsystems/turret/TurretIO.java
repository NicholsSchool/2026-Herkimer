package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface TurretIO {

    public static class TurretIOInputs{
        public double turretAngle = 0.0;
        public double redirectorVelocity  = 0.0;
        public double redirectorPower = 0.0;
        public double shooterVelocity = 0.0;
        public boolean magnetState = true;
        public double rawTurretAngle = 0.0;
        public double tagDistance = 0.0;
        public double tagX = 0.0;
        public Pose2D aprilTagPos = new Pose2D(DistanceUnit.INCH, 0 ,0, AngleUnit.DEGREES, 0);
    }

    public void updateInputs(TurretIOInputs inputs);
    public void resetTurretEncoder(TurretIOInputs inputs);
    public void shooterSetVelocity(double velocity);
    public void redirectorSetVelocity(double velocity);
    public void turretSetPower(double power);




}
