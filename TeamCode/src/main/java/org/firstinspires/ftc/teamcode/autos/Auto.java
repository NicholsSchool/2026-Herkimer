package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.LightManager;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIOReal;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIOReal;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "THE AUTO", group = "Auto")
public class Auto extends LinearOpMode{


    public Drivetrain drivetrain;
    public Intake intake;
    public Turret turret;
    public Vision vision;

    private boolean isRed = false;
    private boolean isAudience = false;
    private boolean isLeaveAuto = false;

    List<Runnable> periodicSet = new ArrayList<>();
    List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();

    @Override
    public void runOpMode() {

        PoseEstimator.init(hardwareMap, new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0), false, true);
        LightManager.inititalize(hardwareMap);
        turret = new Turret(new TurretIOReal(hardwareMap, isRed));

        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap),  hardwareMap);

        intake = new Intake(new IntakeIOReal(hardwareMap));

        AutoUtil.supplyOpModeActive(this::opModeIsActive);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        while (opModeInInit()) {
            if (gamepad2.aWasPressed()) isRed = !isRed;
            telemetry.addLine("Auto alliance side" + (isRed ? "RED" : "BLUE"));
            if (gamepad2.bWasPressed()) isAudience = !isAudience;
            telemetry.addLine("Auto side" + (isAudience ? "AUDIENCE SIDE" : "GOAL SIDE"));
            if (gamepad2.xWasPressed()) isLeaveAuto = !isLeaveAuto;
            telemetry.addLine("Just an Auto Leave" + (isLeaveAuto ? "LEAVE" : "NORMAL"));
            telemetry.update();

            if (PoseEstimator.getPinpointStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
                LightManager.GoBildaLights.setLights(new double[]{LightManager.LightConstants.Green, LightManager.LightConstants.Green, LightManager.LightConstants.Green});
            }
        }

        if (isRed) {turret.setTagID(24);} else {turret.setTagID(20);}

        if(!isAudience) {
            PoseEstimator.setPosition(allianceFlip(isRed, new Pose2D(DistanceUnit.METER, -1.6, -1, AngleUnit.DEGREES, 0)));
        }else{
            PoseEstimator.setPosition(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 64, -24, AngleUnit.DEGREES, 0)));
        }

        LightManager.GoBildaLights.setLights(new double[]{0, 0, 0});

        periodicSet.add(drivetrain::periodic);
        periodicSet.add(PoseEstimator::periodic);
        periodicSet.add(turret::periodic);
        periodicSet.add(intake::periodic);

        periodicSet.add(() -> telemetry.addLine(AutoUtil.getLoopStatesReadout()));
        periodicSet.add(() -> drivetrain.sendDashboardPacket(dashboard));
        periodicSet.add(() -> telemetry.addData("Drive Error", drivetrain.getDrivePIDError().magnitude()));
        periodicSet.add(() -> telemetry.addData("Pose", drivetrain.getPose().toString()));
        periodicSet.add(() -> telemetry.update());


        if(isAudience){
            //this aim doesnt seem to be happening for some reason
            aim();
            shoot();
            intakeRow(3);
            driveToShootFar();
            aim();
            compress();
            shoot();
            intakeRow(2);
            driveToShootFar();
            aim();
            compress();
            shoot();
            intakeRow(1);
            driveToShootFar();
            aim();
            compress();
            shoot();
        }else if(isLeaveAuto){

        }else{
            driveToShoot();
            aim();
            shoot();
            intakeRow(1);
            driveToShoot();
            aim();
            compress();
            shoot();
            intakeRow(2);
            lastDriveToShoot();
            aim();
            compress();
            shoot();
            intakeRow(3);
            driveToShoot();
            aim();
            compress();
            shoot();
        }




//        //drive back to shoot
//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -8, -12, AngleUnit.DEGREES, 45)));
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
//        actionSet.clear();
//
//        //kick anything back down rq
//        intake.kickerGO(-0.5);
//        intake.intakeGO(0.7);
//        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 0.5);
//        intake.kickerGO(0);
//        intake.intakeGO(0);
//
//        //shoot
//        actionSet.add(turret::autoAim);
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
//        actionSet.clear();
//
//        intake.kickerGO(0.7);
//        intake.intakeGO(0.5);
//        turret.runShooterForDistance();
//        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 3);
//        turret.setShooterVelocity(0);
//        intake.kickerGO(0);
//        intake.intakeGO(0);
//
//        //drive to second intake pos
//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -12, -40, AngleUnit.DEGREES, -90)));
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
//        actionSet.clear();
//
//        //drive and intake episode 2
//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -12, -65, AngleUnit.DEGREES, -90)));
//        intake.intakeGO(0.5);
//        turret.setShooterVelocity(-0.3);
//        intake.kickerGO(0.5);
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
//        actionSet.clear();
//
//        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 1);
//        intake.intakeGO(0);
//        turret.setShooterVelocity(0);
//        intake.kickerGO(0);
//
//        //drive to shoot pos
//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -10, -14, AngleUnit.DEGREES, 45)));
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
//        actionSet.clear();
//
//        actionSet.add(turret::autoAim);
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
//        actionSet.clear();
//
//        intake.kickerGO(0.7);
//        intake.intakeGO(0.5);
//        turret.runShooterForDistance();
//        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 3);
//        turret.setShooterVelocity(0);
//        intake.kickerGO(0);
//        intake.intakeGO(0);
//
//        //drive to third intake pos
//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 36, -40, AngleUnit.DEGREES, -90)));
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
//        actionSet.clear();
//
//        //drive and intake ONE MORE TIME!
//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 36, -65, AngleUnit.DEGREES, -90)));
//        intake.intakeGO(0.5);
//        intake.kickerGO(0.5);
//        turret.setShooterVelocity(-0.3);
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
//        intake.intakeGO(0);
//        turret.setShooterVelocity(0);
//        intake.kickerGO(0);
//        actionSet.clear();
//
//        //drive to shoot
//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -8, -12, AngleUnit.DEGREES, 45)));
//        intake.intakeGO(0.7);
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
//        actionSet.clear();
//
//        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 1);
//        intake.intakeGO(0);
//
//        //shoot
//        actionSet.add(turret::autoAim);
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
//        actionSet.clear();
//
//        intake.kickerGO(0.7);
//        intake.intakeGO(0.5);
//        turret.runShooterForDistance();
//        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 3);
//        turret.setShooterVelocity(0);
//        intake.kickerGO(0);
//        intake.intakeGO(0);
//
    }

    //TODO: THIS POSITION WAS CHANGED FROM -10 -14 to -24 -24 FOR SHOOTER, CHANGE IF SHOOTER IS FIXED

    public void driveToShoot(){
        actionSet.clear();

        intake.intakeGO(0.3);
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, -24, -24, AngleUnit.DEGREES, 0))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 4);
        actionSet.clear();
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, -24, -24, AngleUnit.DEGREES, 45))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1);

        actionSet.clear();
    }

    //ITS THE FINAL COUNTDOWWNN
    //we need a diff position so we end with the auto leave points
    public void lastDriveToShoot(){
        actionSet.clear();

        intake.intakeGO(0.3);
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, -36, -14, AngleUnit.DEGREES, 0))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 4);
        actionSet.clear();
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, -36, -14, AngleUnit.DEGREES, 45))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1);

        actionSet.clear();
    }


    public void driveToShootFar(){
        actionSet.clear();

        intake.intakeGO(0.3);
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 52, -14, AngleUnit.DEGREES, 0))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 4);
        actionSet.clear();
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 52, -14, AngleUnit.DEGREES, 45))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1);

        actionSet.clear();
    }

    public void leaveTriangle(){
        actionSet.clear();
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 48, -24, AngleUnit.DEGREES, 0))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 4);
        actionSet.clear();
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 48, -24, AngleUnit.DEGREES, 45))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1);

        actionSet.clear();
    }

    public void aim() {
        actionSet.clear();

        actionSet.add(turret::autoAim);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.2);

        actionSet.clear();
    }

    public void compress() {
        intake.kickerGO(0.5);
        intake.intakeGO(0.5);
        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 0.05);
        intake.kickerGO(0);
        intake.intakeGO(0);
    }

    public void shoot() {
        List<Runnable> shootSet = new ArrayList<>(periodicSet);
//        shootSet.add(() -> turret.redirectorAimAtDistance());
        shootSet.add(() -> {
                    turret.autoAccelerate();
                    LightManager.LEDStrip.setRPMLights(-turret.getShooterVelocity(), -turret.getAcceleratorSetpoint());
                    if (turret.getShooterVelocity() <= turret.getAcceleratorSetpoint() || Math.abs(turret.getShooterVelocity() - turret.getAcceleratorSetpoint()) < TurretConstants.SHOOT_SPEED_TOLERANCE) {
                        intake.kickerGO(-0.9);
                        intake.intakeGO(1);
                    } else {
                        intake.kickerGO(0);
                        intake.intakeGO(0);
                    }
                });
        AutoUtil.runTimedLoop(shootSet, TimeUnit.SECONDS, 5.5);
        turret.setShooterVelocity(0);
        turret.redirectorSetVelocity(0);
    }

    public void intakeRow(int row) {
        actionSet.clear();

        double rowX = -36 + (24 * row);

        //Go To Row
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, rowX, -30, AngleUnit.DEGREES, -90))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.2);
        actionSet.clear();

        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 1);

        //Drive and Intake
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, rowX, -67, AngleUnit.DEGREES, -90)), 0.8));
        intake.intakeGO(0.9);
        turret.setShooterVelocity(-1);
        intake.kickerGO(-0.5);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.0);
        actionSet.clear();

        intake.intakeGO(0);
        turret.setShooterVelocity(0);
        intake.kickerGO(0);

        //back up
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, rowX, -30, AngleUnit.DEGREES, -90))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.2);
        actionSet.clear();

        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 1);
    }

    public void openGate() {
        actionSet.clear();

        //drive to gate opening pos
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 0, -50, AngleUnit.DEGREES, 0))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
        actionSet.clear();

        //drive forward into gate
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 0, -65, AngleUnit.DEGREES, 0))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 2);

        actionSet.clear();
    }

    public void delay(TimeUnit timeUnit, double time){
        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 2);
    }

    private Pose2D allianceFlip(boolean isRed, Pose2D pose){
        return PoseEstimator.allianceFlip(isRed, pose);
    }

}
