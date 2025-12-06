package org.firstinspires.ftc.teamcode.subsystems.intake;



public interface IntakeIO {


    public static class IntakeIOInputs{

    }

    public default void updateInputs(IntakeIO.IntakeIOInputs inputs) {};

    public default void setIntakePower (double power) {};
    public default void setKickerPower (double power) {};
    public default int getTurretEncoder() {return 0;};
    public default void setTurretEncoder(){};

}
