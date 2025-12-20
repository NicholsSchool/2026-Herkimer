package org.firstinspires.ftc.teamcode.autos;

import androidx.appcompat.widget.AppCompatEditText;

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

@Autonomous(name = "12 Artifact BLUE", group = "Auto")
public class BlueAuto12 extends LinearOpMode{


    public Drivetrain drivetrain;
    public Intake intake;
    public Turret turret;
    public Vision vision;


    @Override
    public void runOpMode() {

        PoseEstimator.init(hardwareMap, new Pose2D(DistanceUnit.METER, -1.6, -0.305, AngleUnit.DEGREES, 0), false);
        LightManager.inititalize(hardwareMap);
        turret = new Turret(new TurretIOReal(hardwareMap));

        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap),  hardwareMap);

        intake = new Intake(new IntakeIOReal(hardwareMap));

        AutoUtil.supplyOpModeActive(this::opModeIsActive);


        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        telemetry.update();
        telemetry.addData("drive magnitude", 0.0);
        telemetry.update();

        PoseEstimator.waitForPinpointInit(this::opModeIsActive);

        LightManager.setLights(new double[]{LightManager.LightConstants.Purple, LightManager.LightConstants.Purple, LightManager.LightConstants.Purple});

        waitForStart();

        LightManager.setLights(new double[]{0, 0, 0});

        List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();
        List<Runnable> periodicSet = new ArrayList<>();

        periodicSet.add(drivetrain::periodic);
        periodicSet.add(PoseEstimator::periodic);
        periodicSet.add(turret::periodic);
        periodicSet.add(intake::periodic);
        periodicSet.add(() -> {
            if (turret.aimTagDetected) {
                LightManager.setTopLight(LightManager.LightConstants.Green);
            } else {
                LightManager.setTopLight(0);
            }
        });

        periodicSet.add(() -> telemetry.addLine(AutoUtil.getLoopStatesReadout()));
        periodicSet.add(() -> drivetrain.sendDashboardPacket(dashboard));
        periodicSet.add(() -> telemetry.addData("Drive magnitude", drivetrain.getDrivePIDError()));
        periodicSet.add(() -> telemetry.addData("Pose", drivetrain.getPose().toString()));
        periodicSet.add(() -> telemetry.addData("Drive magnitude", drivetrain.getDrivePIDError()));
        periodicSet.add(() -> telemetry.update());

        //drive to shooting pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 4);
        actionSet.clear();
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 45)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1);
        actionSet.clear();

        //shoot
        actionSet.add(turret::autoAim);
        turret.runShooterForDistance();
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.5);
        actionSet.clear();

        intake.kickerGO(-0.5);
        intake.intakeGO(0.5);
        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 0.5);
        intake.kickerGO(0);
        intake.intakeGO(0);

        intake.kickerGO(1);
        intake.intakeGO(1);
        turret.runShooterForDistance();
        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 4);
        turret.setShooterVelocity(0);
        intake.kickerGO(0);
        intake.intakeGO(0);

        //drive to intake pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -12, -32, AngleUnit.DEGREES, -90)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3);
        actionSet.clear();

        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 1);

        //drive and intake
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -12, -60, AngleUnit.DEGREES, -90), 0.4));
        intake.intakeGO(0.5);
        turret.setShooterVelocity(-1);
        intake.kickerGO(0.5);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 2);
        actionSet.clear();

        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 1);
        intake.intakeGO(0);
        turret.setShooterVelocity(0);
        intake.kickerGO(0);

        //drive to gate opening pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, -50, AngleUnit.DEGREES, 0)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
        actionSet.clear();

        //drive forward into gate
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 0, -65, AngleUnit.DEGREES, 0)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 2);
        actionSet.clear();

        //drive back to shoot
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -8, -12, AngleUnit.DEGREES, 45)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //kick anything back down rq
        intake.kickerGO(-0.5);
        intake.intakeGO(0.5);
        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 0.5);
        intake.kickerGO(0);
        intake.intakeGO(0);

        //shoot
        actionSet.add(turret::autoAim);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
        actionSet.clear();

        intake.kickerGO(0.7);
        intake.intakeGO(0.5);
        turret.runShooterForDistance();
        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 3);
        turret.setShooterVelocity(0);
        intake.kickerGO(0);
        intake.intakeGO(0);

        //drive to second intake pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -12, -40, AngleUnit.DEGREES, -90)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //drive and intake episode 2
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -12, -65, AngleUnit.DEGREES, -90)));
        intake.intakeGO(0.5);
        turret.setShooterVelocity(-0.3);
        intake.kickerGO(0.5);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 1);
        intake.intakeGO(0);
        turret.setShooterVelocity(0);
        intake.kickerGO(0);

        //drive to shoot pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -8, -12, AngleUnit.DEGREES, 45)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        actionSet.add(turret::autoAim);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
        actionSet.clear();

        intake.kickerGO(0.7);
        intake.intakeGO(0.5);
        turret.runShooterForDistance();
        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 3);
        turret.setShooterVelocity(0);
        intake.kickerGO(0);
        intake.intakeGO(0);

        //drive to third intake pos
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 36, -40, AngleUnit.DEGREES, -90)));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        //drive and intake ONE MORE TIME!
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 36, -65, AngleUnit.DEGREES, -90)));
        intake.intakeGO(0.5);
        intake.kickerGO(0.5);
        turret.setShooterVelocity(-0.3);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        intake.intakeGO(0);
        turret.setShooterVelocity(0);
        intake.kickerGO(0);
        actionSet.clear();

        //drive to shoot
        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -8, -12, AngleUnit.DEGREES, 45)));
        intake.intakeGO(0.7);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 5);
        actionSet.clear();

        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 1);
        intake.intakeGO(0);

        //shoot
        actionSet.add(turret::autoAim);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
        actionSet.clear();

        intake.kickerGO(0.7);
        intake.intakeGO(0.5);
        turret.runShooterForDistance();
        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 3);
        turret.setShooterVelocity(0);
        intake.kickerGO(0);
        intake.intakeGO(0);

    }
}
