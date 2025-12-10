package org.firstinspires.ftc.teamcode.testTeleops.Scope;

public interface ArmIO{
    public static class ArmIOInputs{
        public double angleRad = 0.0;
    }

    public default void updateInputs(ArmIOInputs inputs) {};
    public default void setPower(double power) {};

}
