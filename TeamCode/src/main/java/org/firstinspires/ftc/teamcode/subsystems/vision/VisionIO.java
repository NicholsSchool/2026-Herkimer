package org.firstinspires.ftc.teamcode.subsystems.vision;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public interface VisionIO {
    public static class VisionIOInputs{
        public double botYaw = 0.0;
        public double botX = 0.0;
        public double botY = 0.0;
        public double turretAngle = Math.PI / 2;
        public double distanceFromGoal = 0.0;
    }

    public void updateInputs(VisionIOInputs inputs);
}
