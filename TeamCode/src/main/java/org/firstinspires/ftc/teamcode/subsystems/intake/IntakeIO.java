package org.firstinspires.ftc.teamcode.subsystems.intake;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIO;

public interface IntakeIO {


    public static class IntakeIOInputs{

    }

    public default void updateInputs(IntakeIO.IntakeIOInputs inputs) {};

    public default void setIntakePower (double power) {};

}
