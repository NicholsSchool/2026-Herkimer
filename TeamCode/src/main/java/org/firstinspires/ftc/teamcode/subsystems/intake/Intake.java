package org.firstinspires.ftc.teamcode.subsystems.intake;

import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIO;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;

public class Intake extends SubsystemBase implements IntakeConstants {

    private IntakeIO io;
    private final IntakeIO.IntakeIOInputs inputs = new IntakeIO.IntakeIOInputs();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
