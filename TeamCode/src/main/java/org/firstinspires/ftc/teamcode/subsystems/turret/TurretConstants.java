package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface TurretConstants {

        /**
         * The amount of base power the Turret needs to overcome friction
         * */
        public double turretFeedForward = 0.073;

        public double shootingMinRange = 1.5;
        public double shootingMaxRange = 2.4;

        public double turretMax = 1.75;
        public double turretMin = -1.31;

        public double greenLight = 0.45;

        public double mechanicalStopIn = 0.8;
        public double mechanicalStopOut = 1.0;

        public double rotationalPrediction = 0.29;
        public double rotationTranslationPrediction = -0.25;

        public double acceleratorSetpoint = 2200;

        public int DEFAULT_TAGID = 24;

        public int blueTagID = 20;
        public int redTagID = 24;

        public int turretAimThresholdDegrees = 2;

        public Pose2D redTagPos = new Pose2D(DistanceUnit.INCH, -68, 68, AngleUnit.DEGREES, 0);
        public Pose2D blueTagPos = new Pose2D(DistanceUnit.INCH, -68, -68, AngleUnit.DEGREES, 0);

        /**
         * Tolerance for deciding if accelerator speed is great enough for kicker to feed artifacts in Auto
         * */
        double SHOOT_SPEED_TOLERANCE = 200;
        /**
         * Tolerance for deciding if accelerator speed is great enough for kicker to feed artifacts in TeleOp
         * */
        double SHOOT_SPEED_TOLERANCE_TELE = 120;

        public static double kTP = 0.1, kTI = 0.0, kTD = 0.0;
}