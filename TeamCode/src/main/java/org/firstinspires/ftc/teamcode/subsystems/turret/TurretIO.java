package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface TurretIO {

    public static class TurretIOInputs{
        public double turretAngle = Math.PI / 2;
        public double redirectorAngle  = 0.0;
        public double shooterVelocity = 0.0;
        public boolean magnetState = true;
        public double rawTurretAngle = 0.0;
        public double tagDistance = 0.0;
        public double tagX = 0.0;
        public Pose2D aprilTagPos = new Pose2D(DistanceUnit.INCH, 0 ,0, AngleUnit.DEGREES, 0);
    }

    public void updateInputs(TurretIOInputs inputs);
    public void shooterSetVelocity(double velocity);
    public void redirectorSetPosition(double angle);
    public void turretSetPower(double power);




}
