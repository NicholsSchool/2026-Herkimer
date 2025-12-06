package org.firstinspires.ftc.teamcode.subsystems.turret;

public interface TurretIO {

    public static class TurretIOInputs{

        public double artifactAcceleratorVelocity = 0.0;
        public double tagDistance = 0.0;
        public double offset = 0.0;
        public int turretPos = 0;
        public double redirectorPos = 41;
        public boolean magnetState = true;
    }

    public default void updateInputs(TurretIO.TurretIOInputs inputs) {};
    public default void setPowerTurretTurner(double power) {};
    public default void setPosRapidRedirector(double pos) {};
    public default void setVelocityArtifactAccelerator(double velocity) {};



}
