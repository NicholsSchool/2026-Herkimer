package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;

public class DrivetrainIOReal implements DrivetrainIO, DrivetrainConstants {

    DcMotorEx bR, bL, fR, fL;

    public DrivetrainIOReal(){

    }

    @Override
    public void updateInputs (DrivetrainIO.DrivetrainIOInputs inputs){

    }

}
