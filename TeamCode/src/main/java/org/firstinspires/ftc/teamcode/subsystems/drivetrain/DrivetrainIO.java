package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;

public interface DrivetrainIO {

    public static class DrivetrainIOInputs{

        public double imuHeading = 0.0;
        public double posX = 0.0;
        public double posY = 0.0;

    }

    public default void updateInputs(DrivetrainIO.DrivetrainIOInputs inputs) {};
    public default void setDriveMotorPower(double y, double x, double turn) {};
    public default void setFieldDriveMotorPower(double y, double x, double turn, double headingOffset) {}
    public default void setEggPos(double pos1, double pos2) {};
    public default void resetIMU(){}
    public default void setPinpointPos(Pose2D pose){};
    public default GoBildaPinpointDriver.DeviceStatus imuIsReady() {
        return null;
    };

    //color is a 0-1 position value, colors are given on their gobilda website page
    public default void setBackLightColor(double color) {};

}
