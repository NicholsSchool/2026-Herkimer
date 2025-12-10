package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

public interface DrivetrainIO {

    public static class DrivetrainIOInputs{
        public double imuReading = 0.0;
    }

    public default void updateInputs(DrivetrainIO.DrivetrainIOInputs inputs) {};
    public default void setDriveMotorPower(double y, double x, double turn) {};
    public default void setFieldDriveMotorPower(double y, double x, double turn, double headingOffset) {}
    public default void setEggPos(double pos1, double pos2) {};
    public default void resetIMU(){}

    //color is a 0-1 position value, colors are given on their gobilda website page
    public default void setBackLightColor(double color) {};

}
