package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
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
        PoseEstimator.init(hardwareMap, new Pose2D(DistanceUnit.METER, 1, 1, AngleUnit.DEGREES, 90), false);
        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap), hardwareMap);
        intake = new Intake(new IntakeIOReal(hardwareMap));
        turret = new Turret(new TurretIOReal(hardwareMap));

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
            drivetrain.eggPos(-1,-1);
        }

            //Compact on controller2
        if (gamepad2.y){
            intake.intakeGO(0.5);
            intake.kickerGO(-0.5);
            turret.setShooterVelocity(0);
        }else if(gamepad2.b){
            //intake on controller2
            intake.intakeGO(0.5);
            turret.setShooterVelocity(-1);
            intake.kickerGO(0.5);
        }else if(gamepad2.a){
            //outtake on controller2
            intake.intakeGO(-0.5);
            intake.kickerGO(-0.5);
            turret.setShooterVelocity(0);
        }else if (gamepad2.right_trigger > 0.5) {
            turret.runShooterForDistance();
            intake.intakeGO(.5);
            intake.kickerGO(0.7);
        }else if (gamepad2.right_trigger <= 0.5 && gamepad2.right_trigger > 0.2) {
            turret.runShooterForDistance();
            intake.kickerGO(0);
            intake.intakeGO(0);
        }else{
            //everything off
            turret.setShooterVelocity(0);
            intake.intakeGO(0);
            intake.kickerGO(0);
        }


//        //shoot on controller2
//        if (gamepad2.right_trigger > 0.5) {
//            turret.runShooterForDistance();
//        }else{
//            turret.setShooterVelocity(0);
//        }

        //Always looking at apriltag
        turret.autoAim();
        //adjusting shoot angle between 2 points based on distance from tag
        turret.redirectorAimAtDistance();

    }
}
