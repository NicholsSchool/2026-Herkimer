package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.LightManager;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIOReal;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIOReal;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;

import java.util.Arrays;
import java.util.logging.Logger;

@TeleOp (name = "comptele")
public class CompTeleop extends OpMode {

    public Drivetrain drivetrain;
    public Turret turret;
    public Intake intake;
    private FtcDashboard dashboard;
    private boolean isRed = false;
    public ElapsedTime time;
    public double turretManualOffset = 0.0;

    @Override
    public void init(){
        //LightManager.inititalize(hardwareMap);
        PoseEstimator.init(hardwareMap, new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0), false, false);//TODO: CHANGE B4 COMP PLSSSS DO NOT GO TO COMP WITH THIS TRUE
        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap), hardwareMap);
        intake = new Intake(new IntakeIOReal(hardwareMap));
        turret = new Turret(new TurretIOReal(hardwareMap));
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        turret.resetTurretEncoder();
    }
    @Override
    public void init_loop(){
        if(gamepad2.aWasPressed()){
            isRed = !isRed;
        }
        telemetry.addLine("[G2 A] Teleop Alliance Color: " + (isRed ? "RED" : "BLUE"));
        telemetry.update();
    }

    @Override
    public void start() {

        if (isRed) {
            turret.setTagID(24);
           // LightManager.GoBildaLights.setLights(new double[] {LightManager.LightConstants.Red, LightManager.LightConstants.Red, LightManager.LightConstants.Red});
        } else {
            turret.setTagID(20);
           // LightManager.GoBildaLights.setLights(new double[] {LightManager.LightConstants.Blue, LightManager.LightConstants.Blue, LightManager.LightConstants.Blue});
        }

    }

    @Override
    public void loop(){
        time.reset();
        //update all the subsystems
        turret.periodic();
        drivetrain.periodic();
        intake.periodic();
        PoseEstimator.periodic();

        //kickstand/climb on controller1
        if(gamepad1.x){
            drivetrain.eggPos(0.1,0.1);
        }else if(gamepad1.y){
            drivetrain.eggPos(0.9,0.9);
        }


        if(gamepad1.right_bumper){
            if(isRed) {
                drivetrain.driveToPoseSchedulerless(new Pose2D(DistanceUnit.INCH, 47, -33, AngleUnit.DEGREES, 0), 0.6);
            }else{
                drivetrain.driveToPoseSchedulerless(new Pose2D(DistanceUnit.INCH, 29, 33, AngleUnit.DEGREES, 180), 0.6);
            }
            //29, 33, 180 blue goal side
            //49, 33,0 blue audience side
        }else if(gamepad1.left_bumper){
            if(isRed) {
                drivetrain.driveToPoseSchedulerless(new Pose2D(DistanceUnit.INCH, 29, -33, AngleUnit.DEGREES, 180), 0.6);
            }else{
                drivetrain.driveToPoseSchedulerless(new Pose2D(DistanceUnit.INCH, 47, 33, AngleUnit.DEGREES, 0), 0.6);
            }
        }


        //reset the IMU to reset Field oriented on controller1
        if (gamepad1.dpad_up){
            PoseEstimator.resetIMU();
        }

        if (gamepad1.dpad_down){
            PoseEstimator.resetPoseToAutoStart(isRed);
        }

        telemetry.addData("resets time", time.time());


        if(gamepad2.b){
            //intake on controller2
            turret.moveStopIn();
            intake.intakeGO(-0.7);
            turret.setShooterVelocity(-1);
            intake.kickerGO(.7);
            turret.setShooterVelocityTicks(2200);

        }else if(gamepad2.a){
            //outtake on controller2
            turret.takeStopOut();
            intake.intakeGO(0.5);
            intake.kickerGO(-0.5);
            turret.setShooterVelocityTicks(2200);

        }else if (gamepad2.right_trigger > 0.2) {
            turret.moveStopIn();
            turret.autoAccelerate();
            drivetrain.setDriveMultiplier(0.4);
            if ((Math.abs(turret.getShooterVelocity() - turret.getAcceleratorSetpoint())) < TurretConstants.SHOOT_SPEED_TOLERANCE){
                intake.kickerGO(1);
                turret.takeStopOut();
                intake.intakeGO(-1);
            }else{
                intake.kickerGO(0);
                intake.intakeGO(0);
                turret.setShooterVelocityTicks(2200);
            }
        }else{
            //everything off
            turret.moveStopIn();
            intake.intakeGO(0);
            intake.kickerGO(0);
        }

        if (gamepad1.a){
            drivetrain.setDriveMultiplier(0.5);
        }else if(gamepad2.right_trigger <= 0.2){
            drivetrain.setDriveMultiplier(0.8);
        }

        if(!(gamepad1.left_bumper || gamepad1.right_bumper)) {
            drivetrain.driveField(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, isRed ? -Math.PI / 2 : Math.PI / 2);
        }

        if (gamepad2.dpadUpWasPressed()){
            turretManualOffset = turretManualOffset + 5.0;
        }else if(gamepad2.dpadDownWasPressed()){
            turretManualOffset = turretManualOffset - 5.0;
        }

        if(gamepad2.dpad_left){
            turret.resetTurretEncoder();
            turretManualOffset = 0.0;
        }

        if (gamepad2.left_trigger > 0.2) {
            //turret.turretAutoAim();
            turret.turretAutoAimShootOnTheMove(turretManualOffset);
            Logger.getLogger("CompTeleop Turret").info("Updated PID");
        } else {
            turret.turretSetPower(0);
        }

        telemetry.addData("Turret Aim Error", turret.getAimError(AngleUnit.DEGREES));
        telemetry.addData("Turret Position", turret.getTurretPosition(AngleUnit.DEGREES));
        telemetry.addData("Goal distance", turret.getGoalDistance(DistanceUnit.METER));
        telemetry.addData("Turret Setpoint", turret.getTurretSetpoint(AngleUnit.DEGREES));
        telemetry.addData("turret power", turret.getTurretPower());
        telemetry.addData("Heading", PoseEstimator.getPose().getHeading(AngleUnit.DEGREES));
        telemetry.addData("Hood Position", turret.getHoodAngle());
        telemetry.addData("Shooter Velocity", turret.getShooterVelocity());
        telemetry.addData("Shooter Setpoint", turret.getAcceleratorSetpoint());
        telemetry.addData("full loop time", time.time());
        telemetry.addData("turret manual offset", turretManualOffset);
        telemetry.addData("tube current", intake.getKickerCurrent());
        telemetry.addData("intake current", intake.getIntakeCurrent());

        telemetry.addData("1. pos X", PoseEstimator.getPose().getX(DistanceUnit.INCH));
        telemetry.addData("2. pos Y", PoseEstimator.getPose().getY(DistanceUnit.INCH));
//
//        telemetry.addData("Turret Power", turret.getTurretPIDPower());
        telemetry.addData("Raw Turret Ticks", turret.getRawTurretPos());



        //FTC Dashboard telemetry packet
        drivetrain.sendDashboardPacket(dashboard);

    }
}
