package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIOReal;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIOReal;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


import java.util.logging.Logger;
@Config
@TeleOp (name = "tele")
public class teleop extends OpMode {

    public Drivetrain drivetrain;
    public Turret turret;
    public Intake intake;
    public double increment = 0.8;
    public FtcDashboard dashboard;
    public MultipleTelemetry telemetryDB;
    public TelemetryPacket telemetryPacket;


    @Override
    public void start(){

    }

    @Override
    public void init (){
        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap));
        intake = new Intake(new IntakeIOReal(hardwareMap));
        //supplying the whole intake class bc we need the turret encoder that is in the intake motor port
        turret = new Turret(new TurretIOReal(hardwareMap, intake));

        telemetryDB = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryPacket = new TelemetryPacket();


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
            //turret.adeptAccelerateArtifact();
        }else{
            turret.turretTurn(0);
            turret.accelerateArtifact(0.0);
        }

        if (turret.nabNormal() <= 2.5){
            turret.redirect(0.2);
        }else if(turret.nabNormal() > 2.5){
            turret.redirect(0.05);
        }

        if (gamepad1.dpad_down){
            increment = increment + 0.05;
            turret.redirect(increment);
        }else if (gamepad2.dpad_up){
            increment = increment+ - 0.05;
            turret.redirect(increment);
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

//        if(gamepad1.options){
//            drivetrain.eggPos(0,0);
//        }else if(gamepad1.share){
//            drivetrain.eggPos(1,1);
//        }

//        telemetryDB.addData("redirect servo pose from gamepad", gamepad2.right_stick_x);
        telemetryDB.addData("Distance", turret.nabNormal());
        telemetryDB.addData("Turret position", turret.procurePlatePosition());
        telemetryDB.addData("Turret Shooter Velocity", turret.attainAccelerationAntiderivative());
        telemetryDB.addData("Turret Setpoint", turret.attainAchingAccelerationAntiderivative());
//        telemetryDB.addData("Magnet State (false means turret is at magnet)", turret.magnetState());
        telemetryDB.addData("IMU heading", drivetrain.getIMU());
        telemetryDB.addData("RobotPosX", drivetrain.getPosX());
        telemetryDB.addData("RobotPosY", drivetrain.getPosY());
        telemetryDB.addData("collar pos", increment);
        telemetryDB.addData("aim error", turret.aimError());

        telemetry.addData("raw tag distance", turret.rawTagDistance());

        telemetryDB.update();






    }


}
