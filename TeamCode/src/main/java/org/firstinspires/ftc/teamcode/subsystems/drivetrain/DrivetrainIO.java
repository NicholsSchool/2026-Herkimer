package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.firstinspires.ftc.teamcode.tests.Scope.ArmIO;

public interface DrivetrainIO {

    public static class DrivetrainIOInputs{

    }

    public default void updateInputs(DrivetrainIO.DrivetrainIOInputs inputs) {};

}
