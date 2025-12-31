package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface TurretConstants {
        public int frameWidth = 1280;

        public int DEFAULT_TAGID = 24;

        public Pose2D aprilTagPos = new Pose2D(DistanceUnit.INCH, 0 ,0, AngleUnit.DEGREES, 0);
}