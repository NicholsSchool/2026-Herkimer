package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
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
public class CompTeleop extends OpMode implements TeleOpConstants {

    public Drivetrain drivetrain;
    public Turret turret;
    public Intake intake;
    private FtcDashboard dashboard;
    private boolean isRed = false;
    public ElapsedTime time;
    public double turretManualOffset = 0.0;

    @Override
    public void init(){
        PoseEstimator.init(hardwareMap, new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0), false, false);
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

            turret.setTagID(redTagID);

        } else {

            turret.setTagID(blueTagID);

        }

        turret.setShooterVelocityTicks(defaultShooterVelocity);

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

            drivetrain.eggPos(eggPos1,eggPos1);

        }else if(gamepad1.y){

            drivetrain.eggPos(eggPos2,eggPos2);

        }

        if(gamepad1.right_bumper){

            if(isRed) {

                drivetrain.driveToPoseSchedulerless(redFarBase, teleOpDriveToPosSpeed);

            }else{

                drivetrain.driveToPoseSchedulerless(redCloseBase, teleOpDriveToPosSpeed);

            }

        }else if(gamepad1.left_bumper){

            if(isRed) {

                drivetrain.driveToPoseSchedulerless(blueCloseBase, teleOpDriveToPosSpeed);

            }else{

                drivetrain.driveToPoseSchedulerless(blueFarBase, teleOpDriveToPosSpeed);

            }
        }else if(gamepad1.b){

        if(isRed) {

            drivetrain.driveToPoseSchedulerless(redShootingPos, teleOpDriveToPosSpeed);

        }else{

            drivetrain.driveToPoseSchedulerless(blueShootingPos, teleOpDriveToPosSpeed);

        }
    }

        if (gamepad2.back){

            PoseEstimator.resetPoseToAutoStart(isRed);

        }

        if(gamepad2.b){

            //intake on controller2

            turret.moveStopIn();
            intake.intakeGO(intakeIntakeSpeed);
            intake.kickerGO(kickerIntakeSpeed);

        }else if(gamepad2.a){

            //outtake on controller2

            turret.takeStopOut();
            intake.intakeGO(intakeOuttakeSpeed);
            intake.kickerGO(kickerOuttakeSpeed);

        }else if (gamepad2.right_trigger > triggerThreshold) {

            turret.moveStopIn();
            turret.autoAccelerate();
            drivetrain.setDriveMultiplier(driveLowGear);

            if ((Math.abs(turret.getShooterVelocity() - turret.getAcceleratorSetpoint())) < TurretConstants.SHOOT_SPEED_TOLERANCE_TELE){

                intake.kickerGO(kickerShootSpeed);
                turret.takeStopOut();
                intake.intakeGO(intakeShootSpeed);

            }else{

                intake.kickerGO(0);
                intake.intakeGO(0);

            }

        }else if(gamepad2.x){

            turret.takeStopOut();
            intake.kickerGO(safeSpotKicker);
            intake.intakeGO(safeSpotIntake);
            turret.hoodSetServoPosition(safeSpotHood);
            //1.9 m away

        }else{

            //everything off

            turret.moveStopIn();
            intake.intakeGO(0);
            intake.kickerGO(0);

        }

        if (gamepad1.a){

            drivetrain.setDriveMultiplier(driveLowGear);

        }else if(gamepad2.left_bumper){

            drivetrain.setDriveMultiplier(driveLowGear);

        }else if(gamepad2.right_trigger <= triggerThreshold){

            drivetrain.setDriveMultiplier(driveNormalGear);

        }

        if(!(gamepad1.left_bumper || gamepad1.right_bumper || gamepad1.b)) {

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

        if (gamepad2.left_trigger > triggerThreshold) {

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

        telemetry.addData("Raw Turret Ticks", turret.getRawTurretPos());

        //FTC Dashboard telemetry packet
        drivetrain.sendDashboardPacket(dashboard);


    }
}
