package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface TurretConstants {
        public int frameWidth = 1280;

        public int DEFAULT_TAGID = 24;

        public Pose2D redTagPos = new Pose2D(DistanceUnit.METER, -1.41, 1.43, AngleUnit.DEGREES, 0);
        public Pose2D blueTagPos = new Pose2D(DistanceUnit.METER, -1.41, -1.43, AngleUnit.DEGREES, 0);
}