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
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "no scheduler")
public class Autowithoutscheduler extends LinearOpMode {


    Turret turret;

    public void runOpMode() throws InterruptedException {

        PoseEstimator.init(hardwareMap, new Pose2D(DistanceUnit.METER, -1.6, -1, AngleUnit.DEGREES, 0), false, true);

        turret = new Turret(new TurretIOReal(hardwareMap));
        turret.resetTurretEncoder();

        waitForStart();

        turret.periodic();
        PoseEstimator.periodic();

        // turret.turretSetAngle(45, AngleUnit.DEGREES);
//        turret.turretAutoAimShootOnTheMove();
//        while(!turret.turretAtGoal()){
//            turret.periodic();
//            PoseEstimator.periodic();
////            turret.turretSetAngle(45, AngleUnit.DEGREES);
//            turret.turretAutoAimShootOnTheMove();
//        }
//    }
    }
}
