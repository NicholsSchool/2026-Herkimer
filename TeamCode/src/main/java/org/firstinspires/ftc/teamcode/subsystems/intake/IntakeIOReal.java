package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIO;

public class IntakeIOReal implements IntakeIO, IntakeConstants {

    DcMotorEx intakeMotor, kickerMotor;


    public IntakeIOReal(){

    }

    @Override
    public void updateInputs (IntakeIO.IntakeIOInputs inputs){

    }

}
