package org.firstinspires.ftc.teamcode.tests.Scope;

import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

public class Arm extends SubsystemBase {
    private ArmIO io;
    private final ArmIO.ArmIOInputs inputs = new ArmIO.ArmIOInputs();

    double powerManual = 0.0;
    double powerPID = 0.0;
    double targetPos = 0.0;
    boolean manual = false;

    public Arm(ArmIO io){
        this.io = io;
    }
    @Override
    public void periodic(){
        io.updateInputs(inputs);

        if(!manual){
            powerPID = 0.1 * (targetPos - inputs.angleRad);
            powerManual = 0.0;
        }else{
            powerPID = 0.0;
        }

        io.setPower(powerPID + powerManual);
    }

    public void manualPos(double stickPos){
        if(stickPos > 0.05) {
            manual = true;
            powerManual = stickPos;
            targetPos = inputs.angleRad;
        }else{
            manual = false;
        }
    }

    public void runToPos(double targetPos){
        this.targetPos = targetPos;
        manual = false;
    }
}
