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

        if (gamepad2.dpad_left){
            intake.intakeGO(1);
            intake.kickerGO(1);
        }else if(gamepad2.dpad_right){
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

//        turret.turretTurn(gamepad1.right_trigger - gamepad1.left_trigger);

        if (gamepad1.right_trigger > .5) {
            turret.aimAtApriltag();
        }else{
            turret.turretTurn(0);
        }

        if(gamepad2.right_trigger > 0.5){
            turret.accelerateArtifact(0.7);
        }

        if(gamepad2.left_trigger > 0.5){
            turret.accelerateArtifact(0.8);
        }

        if(gamepad2.dpad_up){
            turret.accelerateArtifact(0.9);
        }

        if(gamepad2.dpad_down){
            turret.accelerateArtifact(1.0);
        }

        if(gamepad2.a){
            turret.rapidRedirect(Math.toRadians(35));
        }else if(gamepad2.b){
            turret.rapidRedirect(Math.toRadians(45));
        }else if(gamepad2.x){
            turret.rapidRedirect(Math.toRadians(75));
        }

        telemetry.addData("redirect servo pose from gamepad", gamepad2.right_stick_x);
        telemetry.addData("Distance", turret.nabNormal());

        turret.periodic();
        drivetrain.periodic();
        intake.periodic();




    }

    @Override
    public void stop (){

    }


}
