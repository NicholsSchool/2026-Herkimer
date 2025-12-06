package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIO;

public class IntakeIOReal implements IntakeIO, IntakeConstants {

    DcMotorEx intakeMotor, kickerMotor;

    Rev2mDistanceSensor beam1, beam2;


    public IntakeIOReal(HardwareMap hwMap){

        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        kickerMotor = hwMap.get(DcMotorEx.class, "kicker");

//        beam1 = hwMap.get(Rev2mDistanceSensor.class, "beam1");
//        beam2 = hwMap.get(Rev2mDistanceSensor.class, "beam2");

    }


    @Override
    public void updateInputs (IntakeIO.IntakeIOInputs inputs){

    }

    @Override
    public void setIntakePower(double power){
        intakeMotor.setPower(power);
    }

    @Override
    public void setKickerPower(double power){
        kickerMotor.setPower(power);
    }

    @Override
    public int getTurretEncoder(){
        return intakeMotor.getCurrentPosition();
    }


    public void setTurretEncoder(){
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
