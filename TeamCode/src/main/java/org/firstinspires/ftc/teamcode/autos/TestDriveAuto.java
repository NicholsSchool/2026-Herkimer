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

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Drivetrain Test", group = "Auto")
public class TestDriveAuto extends LinearOpMode {

    public Drivetrain drivetrain;

    private boolean runAxisTests = true;
    private boolean runDiagonalTests = true;
    private boolean runAngleTests = true;
    private boolean runCompactTest = false;

    @Override
    public void runOpMode() {

        LightManager.inititalize(hardwareMap);

        PoseEstimator.init(hardwareMap,  new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0), false, true);
        
        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap), hardwareMap);
        
        AutoUtil.supplyOpModeActive(this::opModeIsActive);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        telemetry.update();
        telemetry.addData("DRIVE ERROR X", -1.0);
        telemetry.addData("DRIVE ERROR Y", -1.0);
        telemetry.addData("TURN ERROR", -1.0);
        telemetry.addData("AIM ERROR", -1.0);
        telemetry.update();

        while (opModeInInit()) {
            if (gamepad1.aWasPressed()) runAxisTests = !runAxisTests;
            telemetry.addLine("[A] Axial Position Tests " + (runAxisTests ? "ENABLED" : "DISABLED"));
            if (gamepad1.xWasPressed()) runDiagonalTests = !runDiagonalTests;
            telemetry.addLine("[X] Diagonal Position Tests " + (runDiagonalTests ? "ENABLED" : "DISABLED"));
            if (gamepad1.bWasPressed()) runAngleTests = !runAngleTests;
            telemetry.addLine("[B] Angle Tests " + (runAngleTests ? "ENABLED" : "DISABLED"));
            if (gamepad1.yWasPressed()) runCompactTest = !runCompactTest;
            telemetry.addLine("[Y] Angle Tests " + (runCompactTest ? "ENABLED" : "DISABLED"));
            telemetry.update();
        }

        waitForStart();
        List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();
        List<Runnable> periodicSet = new ArrayList<>();

        periodicSet.add(PoseEstimator::periodic);
        periodicSet.add(drivetrain::periodic);

        periodicSet.add(() -> telemetry.addLine(AutoUtil.getLoopStatesReadout()));
        periodicSet.add(() -> drivetrain.sendDashboardPacket(dashboard));
        periodicSet.add(() -> telemetry.addData("DRIVE ERROR X", drivetrain.getDrivePIDError().x));
        periodicSet.add(() -> telemetry.addData("DRIVE ERROR Y", drivetrain.getDrivePIDError().y));
        periodicSet.add(() -> telemetry.addData("TURN ERROR", drivetrain.getTurnPIDError()));
        periodicSet.add(() -> telemetry.addData("POSE", drivetrain.getPose().toString()));
        periodicSet.add(() -> telemetry.update());

        if (runAxisTests) {
             
            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 48, 0, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 48, 24, AngleUnit.DEGREES, 90)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 48, 48, AngleUnit.DEGREES, 90)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();
        }

        if (runDiagonalTests) {

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 24, 48, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 48, 0, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 24, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

        }

        if(runAngleTests) {
             
            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 180)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 360)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();
        }

        if(runCompactTest) {

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 90)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 48, 0, AngleUnit.DEGREES, 180)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();

            actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
            actionSet.clear();
        }
             
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

    }
}
