package org.firstinspires.ftc.teamcode.subsystems.turret;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public interface TurretIO {

    public static class TurretIOInputs{

        public double[] getArtifactAcceleratorVelocity = {0.0, 0.0};
        public double tagDistance = 0.0;
        public double offset = 0.0;
    }

    public default void updateInputs(TurretIO.TurretIOInputs inputs) {};
    public default void setPowerTurretTurner(double power) {};
    public default void setPosRapidRedirector(double pos) {};
    public default void setPowerArtifactAccelerator(double power) {};



}
