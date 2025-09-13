package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Module {
    private DcMotorEx driveMotor;
    private AnalogInput turnEncoder;
    private CRServo turnServo;
    private double turnOffset;
    private double servoP;

    public Module(HardwareMap hwMap, double turnOffset, String hwMotor, String hwServo, String hwEncoder, double servoP){
        driveMotor = hwMap.get(DcMotorEx.class, hwMotor);
        turnEncoder = hwMap.get(AnalogInput.class, hwEncoder);
        turnServo = hwMap.get(CRServoImplEx.class,hwServo);
        this.turnOffset = turnOffset;

        this.servoP = servoP;

        this.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Radians
    private double getTurnServoPos() {
        return turnEncoder.getVoltage() / 3.3 * 2 * Math.PI + turnOffset;
    }

    private void servoToPos(double pos){
        turnServo.setPower(servoP * (getTurnServoPos() - pos));
    }

    public void driveModule(double power, double pos){
        if(Math.abs(getTurnServoPos() - pos) > Math.PI / 2){
            driveMotor.setPower(-power);
            servoToPos(pos + Math.PI);
        }else{
            driveMotor.setPower(power);
            servoToPos(pos);
        }
    }
}
