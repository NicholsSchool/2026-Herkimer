package org.firstinspires.ftc.teamcode.teleops;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface TeleOpConstants {

    Pose2D redFarBase = new Pose2D(DistanceUnit.INCH, 47, -33, AngleUnit.DEGREES, 0);
    Pose2D redCloseBase = new Pose2D(DistanceUnit.INCH, 29, 33, AngleUnit.DEGREES, 180);
    Pose2D blueFarBase = new Pose2D(DistanceUnit.INCH, 47, 33, AngleUnit.DEGREES, 0);
    Pose2D blueCloseBase = new Pose2D(DistanceUnit.INCH, 29, -33, AngleUnit.DEGREES, 180);

    Pose2D redShootingPos = new Pose2D(DistanceUnit.INCH, -16, 18, AngleUnit.DEGREES, 135);
    Pose2D blueShootingPos = new Pose2D(DistanceUnit.INCH, -16, -18, AngleUnit.DEGREES, 225);

    public double teleOpDriveToPosSpeed = 0.6;

    public double intakeIntakeSpeed = -0.7;
    public double intakeOuttakeSpeed = 0.5;
    public double intakeShootSpeed = -0.8;
    public double kickerIntakeSpeed = 0.7;
    public double kickerOuttakeSpeed = -0.5;
    public double kickerShootSpeed = 0.8;

    public double triggerThreshold = 0.2;

    public double driveLowGear = 0.5;
    public double driveNormalGear = 0.8;

    public int blueTagID = 20;
    public int redTagID = 24;

    public double eggPos1 = 0.1;
    public double eggPos2 = 0.9;

    public double defaultShooterVelocity = 2200;

    public double safeSpotIntake = -1.0;
    public double safeSpotKicker = 1.0;
    public double safeSpotHood = 0.454;
}
