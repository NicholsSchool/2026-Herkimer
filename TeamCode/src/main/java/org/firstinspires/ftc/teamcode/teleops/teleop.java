package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIOReal;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIOReal;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;

@TeleOp (name = "teleop")
public class teleop extends OpMode {

    public Drivetrain drivetrain;
    public Turret turret;
    public Intake intake;



    @Override
    public void start(){

    }

    @Override
    public void init (){
        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap));
        turret = new Turret(new TurretIOReal(hardwareMap));
        intake = new Intake(new IntakeIOReal(hardwareMap));

    }

    @Override
    public void loop (){
        drivetrain.drive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        //turret.reticleRapidRedirectorRegression();

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

        if (gamepad1.a){
            intake.intakeGO(1);
            intake.kickerGO(1);
        }else if(gamepad1.b){
            intake.intakeGO(-1);
            intake.kickerGO(-1);
        }else {
            intake.intakeGO(0);
            intake.kickerGO(0);
        }


        if(gamepad1.y){turret.accelerateArtifact(1);
        }else{
            turret.accelerateArtifact(0);
        }

        turret.turretTurn(gamepad1.right_trigger - gamepad1.left_trigger);



        if(gamepad2.a){
            turret.rapidRedirect(Math.toRadians(35));
        }else if(gamepad2.b){
            turret.rapidRedirect(Math.toRadians(60));
        }else if(gamepad2.x){
            turret.rapidRedirect(Math.toRadians(75));
        }

        telemetry.addData("redirect servo pose from gamepad", gamepad2.right_stick_x);






    }

    @Override
    public void stop (){

    }


}
