package org.firstinspires.ftc.teamcode.autos;

import static com.qualcomm.hardware.gobilda.GoBildaPinpointDriver.DeviceStatus.NOT_READY;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
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

@Autonomous(name = "12 Artifact BLUE", group = "Auto")
public class BlueAuto12 extends LinearOpMode{


    public Drivetrain drivetrain;
    public Intake intake;
    public Turret turret;
    public Vision vision;


    @Override
    public void runOpMode() {

        turret = new Turret(new TurretIOReal(hardwareMap));

        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap), new Pose2D(DistanceUnit.METER, 1.6, -0.305, AngleUnit.DEGREES, 180), hardwareMap);

        intake = new Intake(new IntakeIOReal(hardwareMap));

        AutoUtil.supplyOpModeActive(this::opModeIsActive);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        telemetry.update();
        telemetry.addData("drive magnitude", 0.0);
        telemetry.addData("initial pose", drivetrain.getInitialPose());
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
        periodicSet.add(() -> telemetry.addData("Drive magnitude", drivetrain.getDriveMagnitude()));
        periodicSet.add(() -> telemetry.addData("POSX", drivetrain.getPosX()));
        periodicSet.add(() -> telemetry.addData("POSY", drivetrain.getPosY()));
        periodicSet.add(() -> telemetry.addData("intial pose", drivetrain.getInitialPose()));
        periodicSet.add(() -> telemetry.addData("Drive magnitude", drivetrain.getDriveMagnitude()));
        periodicSet.add(() -> telemetry.update());

        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //drive to shooting pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, 0.203, -0.305, AngleUnit.DEGREES, 135)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //shoot
//        actionSet.add(() -> shoot);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
        actionSet.clear();

        //drive to intake pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, 0.305, -1.02, AngleUnit.DEGREES, -90)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //drive and intake
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, 0.305, -1.5, AngleUnit.DEGREES, -90)));
//        actionSet.add(() -> intake.intakeGO(.5));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //drive to gate opening pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, 0, -1.27, AngleUnit.DEGREES, 180)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //drive forward into gate
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, 0, -1.5, AngleUnit.DEGREES, 180)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //drive back to shoot
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, 0.203, -0.305, AngleUnit.DEGREES, 135)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //shoot
//        actionSet.add(() -> shoot);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
        actionSet.clear();

        //drive to second intake pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, -0.305, -1.02, AngleUnit.DEGREES, -90)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //drive and intake episode 2
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, -0.305, -1.5, AngleUnit.DEGREES, -90)));
//        actionSet.add(() -> intake.intakeGO(.5));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //drive to shoot pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, 0.203, -0.305, AngleUnit.DEGREES, 135)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //shoot
//        actionSet.add(() -> shoot);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
        actionSet.clear();

        //drive to third intake pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, -0.914, -1.02, AngleUnit.DEGREES, -90)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //drive and intake ONE MORE TIME!
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, -0.914, -1.02, AngleUnit.DEGREES, -90)));
//        actionSet.add(() -> intake.intakeGO(.5));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //drive to shoot
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, 0.203, -0.305, AngleUnit.DEGREES, 135)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //shoot
//        actionSet.add(() -> shoot);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
        actionSet.clear();

    }
}
