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


    @Override
    public void runOpMode() {

        PoseEstimator.init(hardwareMap,  new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0), false);

        turret = new Turret(new TurretIOReal(hardwareMap));

        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap), hardwareMap);

        intake = new Intake(new IntakeIOReal(hardwareMap));

        AutoUtil.supplyOpModeActive(this::opModeIsActive);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        telemetry.update();
        telemetry.addData("DRIVE ERROR", 0.0);
        telemetry.addData("TURN ERROR", 0.0);
        telemetry.update();

        waitForStart();
        List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();
        List<Runnable> periodicSet = new ArrayList<>();

        actionSet.clear();
        periodicSet.clear();

        periodicSet.add(drivetrain::periodic);
        periodicSet.add(turret::periodic);
        periodicSet.add(intake::periodic);

        periodicSet.add(() -> telemetry.addLine(AutoUtil.getLoopStatesReadout()));
        periodicSet.add(() -> drivetrain.sendDashboardPacket(dashboard));
        periodicSet.add(() -> telemetry.addData("DRIVE ERROR", drivetrain.getDrivePIDError()));
        periodicSet.add(() -> telemetry.addData("TURN ERROR", drivetrain.getTurnPIDError()));
        periodicSet.add(() -> telemetry.addData("POSE", drivetrain.getPose().toString()));
        periodicSet.add(() -> telemetry.update());

//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 24, 0, AngleUnit.DEGREES, 0)));
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
//        actionSet.clear();
//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 24, 24, AngleUnit.DEGREES, 0)));
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
//        actionSet.clear();
//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 24, AngleUnit.DEGREES, 0)));
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
//        actionSet.clear();
//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)));
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
//        actionSet.clear();

        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();
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
}
