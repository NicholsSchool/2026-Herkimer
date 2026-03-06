package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIO;

public class IntakeIOReal implements IntakeIO, IntakeConstants {

    DcMotorEx intakeMotor, kickerMotor;

    RevColorSensorV3 cS1, cS2, cS3;


    public IntakeIOReal(HardwareMap hwMap){

        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        kickerMotor = hwMap.get(DcMotorEx.class, "kicker");
//
//        cS1 = hwMap.get(RevColorSensorV3.class, "cS1");
//        cS2 = hwMap.get(RevColorSensorV3.class, "cS2");
//        cS3 = hwMap.get(RevColorSensorV3.class, "cS3");

    }


    @Override
    public void updateInputs (IntakeIO.IntakeIOInputs inputs){
//        inputs.cS1Value = new int[]{cS1.red(),cS1.green(),cS1.blue()};
//        inputs.cS2Value = new int[]{cS2.red(),cS2.green(),cS2.blue()};
//        inputs.cS3Value = new int[]{cS3.red(),cS3.green(),cS3.blue()};
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
