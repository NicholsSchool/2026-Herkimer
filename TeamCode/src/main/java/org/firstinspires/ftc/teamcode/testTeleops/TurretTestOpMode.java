package org.firstinspires.ftc.teamcode.testTeleops;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIO;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIOReal;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIOReal;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIO;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;
import org.firstinspires.ftc.teamcode.subsystems.vision.VisionIOReal;

@TeleOp(name = "turrettest")
public class TurretTestOpMode extends OpMode {

    public Turret turret;
    public Intake intake;
    public Vision vision;
    public Drivetrain drivetrain;


    @Override
    public void init(){
        turret = new Turret(new TurretIOReal(hardwareMap));
        intake = new Intake(new IntakeIOReal(hardwareMap));
        vision = new Vision(new VisionIOReal(hardwareMap));
        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap), new Pose2D(DistanceUnit.METER, 1, 1, AngleUnit.DEGREES, 90), hardwareMap);
    }

    @Override
    public void loop(){

        turret.periodic();
        vision.periodic();
        if (gamepad1.a){
            turret.setActualVelocity(0.69 * (vision.getDistanceFromGoal() - 1) + 5.17699);
        }else{
            turret.setActualVelocity(0.0);
        }



        if (vision.getDistanceFromGoal() > 1.5){
            turret.redirectorSetAngle(0.5708);
        }else if(vision.getDistanceFromGoal() < 1.5){
            turret.redirectorSetAngle(0.3491);
        }

        if (gamepad1.share){
            drivetrain.setPosition();
        }

        //1.55 x 1.55 y

        if (gamepad1.dpad_down){
            intake.intakeGO(.5);
            intake.kickerGO(1);
        }else{
            intake.intakeGO(0);
            intake.kickerGO(0);
        }

        turret.turretSetPower(gamepad1.left_trigger - gamepad1.right_trigger);

        if (gamepad1.dpad_up){
            turret.turretSetAngle(0.0);
        }else if(gamepad1.dpad_left){
            turret.turretSetAngle(Math.PI / 4);
        }else if(gamepad1.dpad_right){
            turret.turretSetAngle(-Math.PI / 4);
        }

        telemetry.addData("turret pos", turret.getTurretPosition());

        telemetry.addData("shooter velocity", turret.getShooterVelocity());

        telemetry.addData("X", vision.getBotX() );
        telemetry.addData("Y", vision.getBotY() );

        telemetry.addData("Pos X", drivetrain.getPosX());
        telemetry.addData("Pos Y", drivetrain.getPosY());
        telemetry.addData("intial pose", drivetrain.getInitialPose());
        telemetry.addData("heading", drivetrain.getIMU());


    }

}

