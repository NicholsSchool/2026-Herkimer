package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

public class Drivetrain extends SubsystemBase implements DrivetrainConstants {

    private DrivetrainIO io;
    private final DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();

    public Drivetrain(DrivetrainIO io){
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void drive(double y, double x, double turn){
        io.setDriveMotorPower(y, x, turn);
    }

    public void driveField(double y, double x, double turn, double headingOffset){io.setFieldDriveMotorPower(y, x ,turn, headingOffset);}
    
    public void lightColor (double color){
        io.setBackLightColor(color);
    }

    public void eggPos(double pos1, double pos2) { io.setEggPos(pos1, pos2); }

    public double imuReading(){
        return inputs.imuReading;
    }

    public void resetIMU(){io.resetIMU();}



}


