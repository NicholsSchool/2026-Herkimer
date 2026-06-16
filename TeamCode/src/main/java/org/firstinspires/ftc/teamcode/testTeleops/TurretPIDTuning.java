package org.firstinspires.ftc.teamcode.testTeleops;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    private FtcDashboard dashboard;


    @Override
    public void init(){
        turret = new Turret(new TurretIOReal(hardwareMap));
        telemetry.setMsTransmissionInterval(50);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

    }

    @Override
    public void loop(){

        turret.periodic();


        if(gamepad2.dpad_up){
            turret.turretSetAngle(45.0, AngleUnit.DEGREES);
        }else if(gamepad2.dpad_down){
            turret.turretSetAngle(90.0, AngleUnit.DEGREES);
        }else if(gamepad2.dpad_left){
            turret.turretSetAngle(20.0, AngleUnit.DEGREES);
        }else if(gamepad2.dpad_right){
            turret.turretSetAngle(0.0, AngleUnit.DEGREES);
        }

        if(gamepad2.a){
            turret.updatePIDController();
        }

        telemetry.addData("error", turret.getAimError(AngleUnit.DEGREES));
        telemetry.addData("position", turret.getTurretPosition(AngleUnit.DEGREES));
        telemetry.addData("setpoint", turret.getTurretSetpoint(AngleUnit.DEGREES));


    }



}
