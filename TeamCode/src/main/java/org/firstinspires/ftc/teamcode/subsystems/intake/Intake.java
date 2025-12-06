package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

public class Intake extends SubsystemBase implements IntakeConstants {

    private IntakeIO io;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();

    public Intake(IntakeIO io){
        this.io = io;
    }


    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public int getTurretPos(){
       return io.getTurretEncoder();
    }

    public void intakeGO (double power){
        io.setIntakePower(power);
    }

    public void kickerGO (double power){
        io.setKickerPower(power);
    }

    public void resetTurretEncoder (){
        io.setTurretEncoder();
    }

    //One must imagine new sequential command...
    public void nemo(){
        
    }

}
