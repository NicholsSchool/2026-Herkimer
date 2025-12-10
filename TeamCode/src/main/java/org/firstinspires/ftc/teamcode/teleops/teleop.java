package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIOReal;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIOReal;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;

import java.util.logging.Logger;
@TeleOp (name = "tele")
public class teleop extends OpMode {

    public Drivetrain drivetrain;
    public Turret turret;
    public Intake intake;

    public double redirectIncrement = 0.8;



    @Override
    public void start(){

    }

    @Override
    public void init (){
        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap));
        intake = new Intake(new IntakeIOReal(hardwareMap));
        //supplying the whole intake class bc we need the turret encoder that is in the intake motor port
        turret = new Turret(new TurretIOReal(hardwareMap, intake));

        drivetrain.resetIMU();


    }

    @Override
    public void loop (){

        turret.periodic();
        drivetrain.periodic();
        intake.periodic();

        drivetrain.driveField(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0);

        if (gamepad1.right_trigger > 0.5) {
            turret.aimAtApriltag();
            turret.autoAccelerateArtifact();
            turret.reticleRapidRedirectorRegression();
        }else{
            turret.turretTurn(0);
            turret.accelerateArtifact(0.0);
        }

        if (gamepad1.dpad_left){
            intake.intakeGO(.5);
            intake.kickerGO(1);
        }else if(gamepad1.dpad_right){
            intake.intakeGO(-.5);
            intake.kickerGO(-1);
        }else {
            intake.intakeGO(0);
            intake.kickerGO(0);
        }

        if(gamepad1.options){
            drivetrain.eggPos(0,0);
        }else if(gamepad1.share){
            drivetrain.eggPos(1,1);
        }

        telemetry.addData("redirect servo pose from gamepad", gamepad2.right_stick_x);
        telemetry.addData("Distance", turret.nabNormal());
        telemetry.addData("joules per coulomb", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addData("Turret position", turret.procurePlatePosition());
        telemetry.addData("Turret Shooter Velocity", turret.attainAccelerationAntiderivative());
        telemetry.addData("Magnet State (false means turret is at magnet)", turret.magnetState());
        telemetry.addData("IMU reading", drivetrain.imuReading());
        telemetry.addData("increment", redirectIncrement);
        Logger.getGlobal().info(String.format("joules per coulomb: %.2f", hardwareMap.voltageSensor.iterator().next().getVoltage()));


    }


}
