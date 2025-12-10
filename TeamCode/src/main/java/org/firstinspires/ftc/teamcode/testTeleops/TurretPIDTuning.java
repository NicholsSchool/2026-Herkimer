package org.firstinspires.ftc.teamcode.testTeleops;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIOReal;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIOReal;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;

import com.acmerobotics.dashboard.FtcDashboard;

@TeleOp (name = "PIDTuning")
public class TurretPIDTuning extends OpMode {

    public Turret turret;
    public Intake intake;
    public Drivetrain drivetrain;

    @Override
    public void init(){
        intake = new Intake(new IntakeIOReal(hardwareMap));
        turret = new Turret(new TurretIOReal(hardwareMap, intake));
        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap));
        telemetry.setMsTransmissionInterval(50);

    }

    @Override
    public void loop(){
        intake.periodic();
        turret.periodic();
        drivetrain.periodic();

        if (gamepad1.right_trigger > 0.5) {
            turret.aimAtApriltag();
        }else{
            turret.turretTurn(0);
        }

        telemetry.addData("error", turret.aimError());


    }



}
