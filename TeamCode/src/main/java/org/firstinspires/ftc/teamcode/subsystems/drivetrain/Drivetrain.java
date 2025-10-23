package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

public class Drivetrain extends SubsystemBase implements DrivetrainConstants {

    private DrivetrainIO io;
    private final DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void drive(double y, double x, double turn){
        io.setDriveMotorPower(y, x, turn);
    }

    public void fieldOrientetedDrive(double y, double x, double turn){

        double fieldX = x * Math.cos(-inputs.imuReading) - y * Math.sin(-inputs.imuReading);
        double fieldY = x * Math.sin(-inputs.imuReading) + y * Math.cos(-inputs.imuReading);
//
//        backRight.setPower(y + x - turn);
//        backLeft.setPower(y - x + turn);
//        frontRight.setPower(y - x - turn);
//        frontLeft.setPower(y + x + turn);


    }



}


