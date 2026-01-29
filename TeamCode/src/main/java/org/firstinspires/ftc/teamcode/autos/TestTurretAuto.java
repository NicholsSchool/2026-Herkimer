package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.LightManager;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIOReal;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "Turret Testing")
public class TestTurretAuto extends LinearOpMode {

    Turret turret;

    @Override
    public void runOpMode() throws InterruptedException {
        LightManager.inititalize(hardwareMap);

        PoseEstimator.init(hardwareMap,  new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0), false, true);

        turret = new Turret(new TurretIOReal(hardwareMap, false));
        turret.resetTurretEncoder();

        AutoUtil.supplyOpModeActive(this::opModeIsActive);

        List<Runnable> periodicSet = new ArrayList<>();
        periodicSet.add(turret::periodic);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        telemetry.update();
        telemetry.addData("ERROR", -1.0);
        telemetry.addData("Time", AutoUtil.getLoopTime());
        telemetry.update();

        waitForStart();

        periodicSet.add(() -> telemetry.addData("ERROR", turret.turretPIDController.getPositionError()));
        periodicSet.add(() -> telemetry.addData("Time", AutoUtil.getLoopTime()));
        periodicSet.add(telemetry::update);


        List<Runnable> periodicWithAngle = new ArrayList<>(periodicSet);
        periodicWithAngle.add(() -> { turret.turretSetAngle(-45, AngleUnit.DEGREES); });
        AutoUtil.runTimedLoop(periodicWithAngle, TimeUnit.SECONDS, 4);

        periodicWithAngle = new ArrayList<>(periodicSet);
        periodicWithAngle.add(() -> { turret.turretSetAngle(0, AngleUnit.DEGREES); });
        AutoUtil.runTimedLoop(periodicWithAngle, TimeUnit.SECONDS, 4);

        periodicWithAngle = new ArrayList<>(periodicSet);
        periodicWithAngle.add(() -> { turret.turretSetAngle(45, AngleUnit.DEGREES); });
        AutoUtil.runTimedLoop(periodicWithAngle, TimeUnit.SECONDS, 4);

        periodicWithAngle = new ArrayList<>(periodicSet);
        periodicWithAngle.add(() -> { turret.turretSetAngle(0, AngleUnit.DEGREES); });
        AutoUtil.runTimedLoop(periodicWithAngle, TimeUnit.SECONDS, 4);


//        periodicWithAngle = new ArrayList<>(periodicSet);
//        periodicWithAngle.add(() -> { turret.turretSetAngle(-90, AngleUnit.DEGREES); });
//        AutoUtil.runTimedLoop(periodicWithAngle, TimeUnit.SECONDS, 2);
//
//        periodicWithAngle = new ArrayList<>(periodicSet);
//        periodicWithAngle.add(() -> { turret.turretSetAngle(0, AngleUnit.DEGREES); });
//        AutoUtil.runTimedLoop(periodicWithAngle, TimeUnit.SECONDS, 2);
//
//        periodicWithAngle = new ArrayList<>(periodicSet);
//        periodicWithAngle.add(() -> { turret.turretSetAngle(90, AngleUnit.DEGREES); });
//        AutoUtil.runTimedLoop(periodicWithAngle, TimeUnit.SECONDS, 2);
//
//        periodicWithAngle = new ArrayList<>(periodicSet);
//        periodicWithAngle.add(() -> { turret.turretSetAngle(0, AngleUnit.DEGREES); });
//        AutoUtil.runTimedLoop(periodicWithAngle, TimeUnit.SECONDS, 2);

    }
}
