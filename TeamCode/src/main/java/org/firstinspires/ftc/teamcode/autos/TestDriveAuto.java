package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Auto Existing Test", group = "Auto")
public class TestDriveAuto extends LinearOpMode {

    public Drivetrain drivetrain;
    public Intake intake;
    public Turret turret;
    public Vision vision;

    private boolean runPositionTests = true;
    private boolean runAngleTests = true;

    @Override
    public void runOpMode() {

        LightManager.inititalize(hardwareMap);

        PoseEstimator.init(hardwareMap,  new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0), false, true);

        turret = new Turret(new TurretIOReal(hardwareMap));

        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap), hardwareMap);

        intake = new Intake(new IntakeIOReal(hardwareMap));

        AutoUtil.supplyOpModeActive(this::opModeIsActive);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        telemetry.update();
        telemetry.addData("DRIVE ERROR", -1.0);
        telemetry.addData("TURN ERROR", -1.0);
        telemetry.addData("AIM ERROR", -1.0);
        telemetry.addData("FLYWHEEL ERROR", turret.desiredVelocity - turret.getShooterVelocity());
        telemetry.update();

        while (opModeInInit()) {
            if (gamepad1.aWasPressed()) runPositionTests = !runPositionTests;
            telemetry.addLine("Position Tests " + (runPositionTests ? "ENABLED" : "DISABLED"));
            if (gamepad1.bWasPressed()) runAngleTests = !runAngleTests;
            telemetry.addLine("Angle Tests " + (runAngleTests ? "ENABLED" : "DISABLED"));
            telemetry.update();
        }

        waitForStart();
        List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();
        List<Runnable> periodicSet = new ArrayList<>();

        periodicSet.add(PoseEstimator::periodic);
        periodicSet.add(drivetrain::periodic);
        periodicSet.add(turret::periodic);
        periodicSet.add(intake::periodic);

        periodicSet.add(() -> telemetry.addLine(AutoUtil.getLoopStatesReadout()));
        periodicSet.add(() -> drivetrain.sendDashboardPacket(dashboard));
        periodicSet.add(() -> telemetry.addData("DRIVE ERROR", drivetrain.getDrivePIDError()));
        periodicSet.add(() -> telemetry.addData("TURN ERROR", drivetrain.getTurnPIDError()));
        periodicSet.add(() -> telemetry.addData("POSE", drivetrain.getPose().toString()));
        periodicSet.add(() -> telemetry.addData("AIM ERROR", turret.aimError));
        periodicSet.add(() -> telemetry.addData("FLYWHEEL ERROR", turret.desiredVelocity - turret.getShooterVelocity()));
        periodicSet.add(() -> telemetry.update());

        if (runPositionTests) {
            drivetrain.resetPID();
            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            drivetrain.resetPID();
            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 48, 0, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            drivetrain.resetPID();
            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 48, 24, AngleUnit.DEGREES, 90)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            drivetrain.resetPID();
            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 48, 48, AngleUnit.DEGREES, 90)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            drivetrain.resetPID();
            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();
        }

        if(runAngleTests) {
            drivetrain.resetPID();
            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            drivetrain.resetPID();
            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 180)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            drivetrain.resetPID();
            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 360)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();
        }
            drivetrain.resetPID();
            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 45)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();


        while (opModeIsActive()){
            telemetry.addData("Turret Aim State", AutoUtil.toString(turret.autoAim()));
            telemetry.addData("RANDOM", Math.random());
            for (Runnable run : periodicSet) {
                run.run();
            }

            if(gamepad1.right_trigger > 0.5){
                intake.intakeGO(0.7);
                intake.kickerGO(1);
                turret.runShooterForDistance();
            } else {
                intake.intakeGO(0);
                intake.kickerGO(0);
                turret.setShooterVelocity(0);
            }
        }

    }
}
