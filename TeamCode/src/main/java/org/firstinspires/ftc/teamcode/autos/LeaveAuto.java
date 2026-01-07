//package org.firstinspires.ftc.teamcode.autos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
//import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
//import org.firstinspires.ftc.teamcode.subsystems.drivetrain.Drivetrain;
//import org.firstinspires.ftc.teamcode.subsystems.drivetrain.DrivetrainIOReal;
//
//import java.util.ArrayList;
//import java.util.List;
//import java.util.concurrent.Callable;
//import java.util.concurrent.TimeUnit;
//
//@Autonomous(name = "LEAVE")
//public class LeaveAuto extends LinearOpMode {
//
//    @Override
//    public void runOpMode() {
//
//        List<Runnable> periodicSet = new ArrayList<>();
//        List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();
//
//        PoseEstimator.init(hardwareMap, new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0), false, true);
//        Drivetrain drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap), hardwareMap);
//
//        periodicSet.add(drivetrain::periodic);
//        periodicSet.add(PoseEstimator::periodic);
//
//        AutoUtil.supplyOpModeActive(this::opModeIsActive);
//
//        waitForStart();
//
//        actionSet.add(() -> drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, -12, 0, AngleUnit.DEGREES, 0)));
//        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 4);
//        actionSet.clear();
//
//    }
//
//}
