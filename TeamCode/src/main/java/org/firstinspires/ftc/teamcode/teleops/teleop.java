package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIOReal;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIOReal;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;

import java.util.logging.Logger;

@TeleOp (name = "teleop")
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
//        drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        drivetrain.driveField(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0);

//        turret.reticleRapidRedirectorRegression();
        if (gamepad1.right_trigger > .5) {
            turret.aimAtApriltag();
        }else{
            turret.turretTurn(0);
        }

//        if (gamepad1.a){
//            intake.intakeGO(1);
//            intake.kickerGO(1);
//        }else{
//            intake.intakeGO(0);
//            intake.kickerGO(0);
//        }
//
//        if (gamepad1.x){
//            turret.accelerateArtifact(0.4);
//        }else{
//            turret.accelerateArtifact(0);
//        } 

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




//        if(gamepad1.y){turret.accelerateArtifact(1);
//        }else{
//            turret.accelerateArtifact(0);
//        }

//        turret.turretTurn(gamepad1.right_trigger - gamepad1.left_trigger);


////
//        if(gamepad2.right_trigger > 0.5){
//            turret.accelerateArtifact(0.7);
//        }
//
//        if(gamepad2.left_trigger > 0.5){
//            turret.accelerateArtifact(0.8);
//        }

//        if(gamepad1.dpad_up){
//            turret.accelerateArtifact(0.9);
//        }

        if(gamepad1.dpad_down){
            turret.autoAccelerateArtifact();
        }else{
            turret.accelerateArtifact(0);
        }

//        if(gamepad1.a){
//            turret.rapidRedirect(8);
//            //.7
//        }else if(gamepad1.b){
//            turret.rapidRedirect(19);
//            //.5
//        }else if(gamepad1.x){
//            turret.rapidRedirect(27);
//            //.3
//        }else if(gamepad1.y){
//            turret.rapidRedirect(41);
//            //0
//        }


        if (gamepad2.dpad_up){
            redirectIncrement = redirectIncrement - .05;
            turret.redirect(redirectIncrement);
        }else if(gamepad2.dpad_down){
            redirectIncrement = redirectIncrement + .05;
            turret.redirect(redirectIncrement);
        }



//        turret.redirect(gamepad2.left_stick_y);


        telemetry.addData("redirect servo pose from gamepad", gamepad2.right_stick_x);
        telemetry.addData("Distance", turret.nabNormal());
        telemetry.addData("joules per coulomb", hardwareMap.voltageSensor.iterator().next().getVoltage());
        telemetry.addData("Turret position", turret.procurePlatePosition());
        telemetry.addData("Turret Shooter Velocity", turret.attainAccelerationAntiderivative());
        telemetry.addData("Magnet State (false means turret is at magnet)", turret.magnetState());
        telemetry.addData("shooter velocity", turret.shooterVelocity());
        telemetry.addData("IMU reading", drivetrain.imuReading());
        telemetry.addData("increment", redirectIncrement);
        Logger.getGlobal().info(String.format("joules per coulomb: %.2f", hardwareMap.voltageSensor.iterator().next().getVoltage()));

        turret.periodic();
        drivetrain.periodic();
        intake.periodic();




    }


}
