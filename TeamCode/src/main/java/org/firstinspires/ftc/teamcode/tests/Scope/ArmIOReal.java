package org.firstinspires.ftc.teamcode.tests.Scope;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmIOReal implements ArmIO{
    DcMotorEx leftArm, rightArm;
    HardwareMap hwMap;

    public ArmIOReal(){
        leftArm = hwMap.get(DcMotorEx.class,"lA");
        rightArm = hwMap.get(DcMotorEx.class, "rA");
    }
    @Override
    public void updateInputs(ArmIO.ArmIOInputs inputs){
        inputs.angleRad = leftArm.getCurrentPosition() / 360.0;
    }
    @Override
    public void setPower(double power){
        leftArm.setPower(power);
        rightArm.setPower(power);
    }

}
