package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIOReal;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIOReal;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;

@TeleOp (name = "comptele")
public class CompTeleop extends OpMode {


    public Drivetrain drivetrain;
    public Turret turret;
    public Intake intake;

    @Override
    public void init(){
        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap));
        intake = new Intake(new IntakeIOReal(hardwareMap));
        //supplying the whole intake class bc we need the turret encoder that is in the intake motor port
        turret = new Turret(new TurretIOReal(hardwareMap, intake));

        drivetrain.resetIMU();
    }

    @Override
    public void loop(){

        //update all the subsystems
        turret.periodic();
        drivetrain.periodic();
        intake.periodic();

        //field-oriented driving on controller1
        drivetrain.driveField(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 0);

        //kickstand/climb on controller1
        if(gamepad1.options){
            drivetrain.eggPos(0,0);
        }else if(gamepad1.share){
            drivetrain.eggPos(1,1);
        }

        //shoot on controller2
        if (gamepad2.right_trigger > 0.5) {
            turret.autoAccelerateArtifact();
        }else{
            turret.accelerateArtifact(0.0);
        }

        //Always looking at apriltag
        turret.aimAtApriltag();
        //always adjusting shoot angle based on distance from tag
        turret.reticleRapidRedirectorRegression();

    }
}
