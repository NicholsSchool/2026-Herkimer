package org.firstinspires.ftc.teamcode.subsystems.intake;

import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

public class Intake extends SubsystemBase implements IntakeConstants {

    private IntakeIO io;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();


    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void intakeGO (double power){
        io.setIntakePower(power);
    }

    public void kickerGO (double power){
        io.setKickerPower(power);
    }

}
