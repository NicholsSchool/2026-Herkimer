package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIO;

public class IntakeIOReal implements IntakeIO, IntakeConstants {

    //intake and indexer motors
    DcMotorEx intakeMotor, kickerMotor;

    //bottom LED for driver feedback
    Servo bottomLight;


    public IntakeIOReal(HardwareMap hwMap){

        intakeMotor = hwMap.get(DcMotorEx.class, "intake");
        kickerMotor = hwMap.get(DcMotorEx.class, "kicker");
        bottomLight = hwMap.get(Servo.class, "bottomLight");

    }


    @Override
    public void updateInputs (IntakeIO.IntakeIOInputs inputs){
        inputs.intakeCurrent = intakeMotor.getCurrent(CurrentUnit.AMPS);
        inputs.kickerCurrent = kickerMotor.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * provides the given power to the intake motor
     * @param power the power supplied to the intake motor
     * */
    @Override
    public void setIntakePower(double power){
        intakeMotor.setPower(power);
    }

    /**
     * provides the given power to the kicker motor
     * @param power the power supplied to the kicker motor
     * */
    @Override
    public void setKickerPower(double power){
        kickerMotor.setPower(power);
    }

    /**
     * Sets the color of the lights based on the Servo position given
     * (the lights are a Servo)
     * @param position The position of the Servo
     * */
    @Override
    public void setLightPosition(double position){
        bottomLight.setPosition(position);
    }

    public void setTurretEncoder(){
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

}
