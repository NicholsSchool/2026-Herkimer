package org.firstinspires.ftc.teamcode.autos;

import android.graphics.Bitmap;

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
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIOReal;
import org.firstinspires.ftc.teamcode.subsystems.vision.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Callable;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "THE AUTO", group = "Auto")
public class Auto extends LinearOpMode{


    public interface ConfigOption {
        static ConfigOption[] values() {
            return null;
        }
    }

    public enum Alliance implements ConfigOption {
        RED,
        BLUE;
    }

    private enum StartPosition implements ConfigOption {
        GOAL,
        AUDIENCE;
    }

    private enum CycleType implements ConfigOption {

        GOAL_SPIKE,
        MID_SPIKE,
        AUDIENCE_SPIKE,
        GATE_CYCLE,
        HP_PRESET,
        HP_JAB,
        SKIP,
        WAIT;
    }

    private enum LeaveOnly implements ConfigOption {
        ENABLED,
        DISABLED;
    }

    public Drivetrain drivetrain;
    public Intake intake;
    public Turret turret;
    public Vision vision;

    public HashMap<String, ConfigOption> options = new HashMap<>();
    public int cycleCount = 0;

    List<Runnable> periodicSet = new ArrayList<>();
    List<Callable<AutoUtil.AutoActionState>> actionSet = new ArrayList<>();

    boolean isRed, isAudience;

    @Override
    public void runOpMode() {

        options.put("Alliance", Alliance.BLUE);
        options.put("Starting Position", StartPosition.GOAL);
        options.put("Cycle 1 Type", CycleType.GOAL_SPIKE);
        options.put("Cycle 2 Type", CycleType.MID_SPIKE);
        options.put("Cycle 3 Type", CycleType.GATE_CYCLE);
        options.put("Cycle 4 Type", CycleType.AUDIENCE_SPIKE);
        options.put("Cycle 5 Type", CycleType.HP_PRESET);
        options.put("Leave only", LeaveOnly.DISABLED);

        PoseEstimator.init(hardwareMap, new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0), false, true);
        LightManager.inititalize(hardwareMap);
        turret = new Turret(new TurretIOReal(hardwareMap));

        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap), hardwareMap);

        intake = new Intake(new IntakeIOReal(hardwareMap));

        AutoUtil.supplyOpModeActive(this::opModeIsActive);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);

        String currentOption = "Alliance";
        List<String> keySet = new ArrayList<>(options.keySet());


        while (opModeInInit()) {

            if (gamepad2.dpadDownWasPressed() || gamepad2.dpadUpWasPressed()) {
                currentOption = keySet.get((keySet.indexOf(currentOption) + (gamepad2.dpad_down ? -1 : 1)) % keySet.size());
            }


            if (gamepad2.aWasPressed()) {
                ConfigOption option = options.get(currentOption);
//                 assert ConfigOption.values() != null;
//                List<ConfigOption> possibleValues = Arrays.asList(ConfigOption.values());
                List<ConfigOption> possibleValues = Collections.emptyList();
                if (option instanceof StartPosition) {
                    possibleValues = Arrays.asList(StartPosition.values());
                } else if (option instanceof Alliance) {
                    possibleValues = Arrays.asList(Alliance.values());
                } else if (option instanceof CycleType) {
                    possibleValues = Arrays.asList(CycleType.values());
                } else if (option instanceof LeaveOnly) {
                    possibleValues = Arrays.asList(LeaveOnly.values());
                }


                options.put(currentOption, possibleValues.get((possibleValues.indexOf(option) + 1) % possibleValues.size()));
            }

            for (int i = 0; i < options.size(); i++) {
                telemetry.addData((keySet.indexOf(currentOption) == i ? ">" : "-") + keySet.get(i), options.get(keySet.get(i)));
            }
            telemetry.update();

        }

        LightManager.LEDStrip.clear();

        isRed = options.get("Alliance") == Alliance.RED;
        isAudience = options.get("Starting Position") == StartPosition.AUDIENCE;

        if (isRed) {
            turret.setTagID(24);
        } else {
            turret.setTagID(20);
        }

        if (!isAudience) {
            // the isRed in the X position is because the red auto DOES NOT WORK unless it has +4 inches. i dont know why. pls send help. :(
            PoseEstimator.setPosition(allianceFlip(isRed, new Pose2D(DistanceUnit.METER, isRed ? -1.5 : -1.6, -1, AngleUnit.DEGREES, 0)));
        } else {
            PoseEstimator.setPosition(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, isRed ? 65 : 64, -24, AngleUnit.DEGREES, 0)));
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

        if (isAudience) {
            driveToShoot();
        }

        if (options.get("Leave only") == LeaveOnly.ENABLED) {

        } else {
            runCycle((CycleType) options.get("Cycle 1 Type"));
            cycleCount = 1;
            runCycle((CycleType) options.get("Cycle 2 Type"));
            cycleCount = 2;
            runCycle((CycleType) options.get("Cycle 3 Type"));
            cycleCount = 3;
            runCycle((CycleType) options.get("Cycle 4 Type"));
            cycleCount = 4;
            runCycle((CycleType) options.get("Cycle 5 Type"));
        }

    }

    private void runCycle(CycleType cycleType) {

        HashMap<String, ConfigOption > cycles = (HashMap<String, ConfigOption>) options.clone();

        cycles.remove("Alliance");
        cycles.remove("Starting Position");
        cycles.remove("Leave only");

        List<ConfigOption> prevCycles = new ArrayList<>(cycles.values()).subList(0, cycleCount - 1);

        if (!prevCycles.contains(CycleType.SKIP) || !prevCycles.contains(CycleType.WAIT)) {

        }

        switch (cycleType) {
            case AUDIENCE_SPIKE:
                intakeRow(3);
                driveToShoot(false, false);
                shoot();
                break;
            case MID_SPIKE:
                intakeRow(2);
                driveToShoot(false, false);
                shoot();
                break;
            case GOAL_SPIKE:
                intakeRow(1);
                driveToShoot(false, false);
                shoot();
                break;
            case GATE_CYCLE:
                intakeFromGate();
                driveToShoot(false,false);
                shoot();
                break;
            case HP_PRESET:
                intakeHumanPlayerPreset();
                driveToShoot(false,false);
                shoot();
                break;
            case HP_JAB:

                break;
            case WAIT:
                delay(TimeUnit.SECONDS, 5);

                break;
            case SKIP:
            default:
                break;
        }

    }

    //TODO: THIS POSITION WAS CHANGED FROM -10 -14 to -24 -24 FOR SHOOTER, CHANGE IF SHOOTER IS FIXED

    public void driveToShoot(boolean inTwoSteps, boolean lastShootPos){
        actionSet.clear();

        if(isAudience){

            if(inTwoSteps) {

                intake.kickerGO(0.5);
                intake.intakeGO(0.5);
                actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, -24, -24, AngleUnit.DEGREES, 0)), 1));
                AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3);
                actionSet.clear();
                actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, -24, -24, AngleUnit.DEGREES, 225)),1));
                AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1);

                actionSet.clear();
            }else if(lastShootPos){

                intake.kickerGO(0.5);
                intake.intakeGO(0.5);
                actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, -36, -18, AngleUnit.DEGREES, 241)),1));
                AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3);

                actionSet.clear();
            }else {
                intake.kickerGO(0.5);
                intake.intakeGO(0.5);
                actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, -24, -24, AngleUnit.DEGREES, 225)),1));
                AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 2);
                actionSet.clear();

            }
        }else{
            actionSet.clear();

            intake.intakeGO(0.3);
            actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 52, -14, AngleUnit.DEGREES, 0))));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 4);
            actionSet.clear();
            actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 52, -14, AngleUnit.DEGREES, 27))));
            AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1);

            actionSet.clear();
        }

    }

    public void driveToIntakeFarMidPoint(){

        actionSet.clear();

        intake.intakeGO(0.3);
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 58, -24, AngleUnit.DEGREES, 0))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 0.5);
        actionSet.clear();

        intake.intakeGO(0.3);
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 58, -24, AngleUnit.DEGREES, 24))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 0.5);
        actionSet.clear();
    }

    public void leaveTriangle(){
        actionSet.clear();
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 48, -24, AngleUnit.DEGREES, 0))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 4);
        actionSet.clear();
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 48, -24, AngleUnit.DEGREES, 0))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1);

        actionSet.clear();
    }

    public void aim() {
        actionSet.clear();

        actionSet.add(turret::autoAim);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1);

        actionSet.clear();
    }

    public void compress() {
        intake.kickerGO(0.5);
        intake.intakeGO(0.5);
        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 0.005);
        intake.kickerGO(0);
        intake.intakeGO(0);
    }

    public void shoot() {
        List<Runnable> shootSet = new ArrayList<>(periodicSet);
//        shootSet.add(() -> turret.redirectorAimAtDistance());
        shootSet.add(() -> {
                    turret.autoAccelerate();
                    LightManager.LEDStrip.setRPMLights(-turret.getShooterVelocity(), -turret.getAcceleratorSetpoint());
                    if (Math.abs(turret.getShooterVelocity() - turret.getAcceleratorSetpoint()) < TurretConstants.SHOOT_SPEED_TOLERANCE) {
                        intake.kickerGO(-1);
                        intake.intakeGO(1);
                    } else {
                        intake.kickerGO(0);
                        intake.intakeGO(0);
                    }
                });
        AutoUtil.runTimedLoop(shootSet, TimeUnit.SECONDS, 1.55);
        LightManager.LEDStrip.clear();
        turret.setShooterVelocity(0);
        turret.redirectorSetVelocity(0);
    }

    public void intakeRow(int row) {
        actionSet.clear();

        double rowX = -36 + (24 * row);
        int[] ySequence;

        if (row == 1) ySequence = new int[]{-28, -60, -28}; // Fix for row 1 going too far
        else {
            ySequence = new int[]{-26, -68, -28};
        }

        //Go To Row
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, isAudience ? rowX + 3.5 : rowX, ySequence[0], AngleUnit.DEGREES, -90))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.2);
        actionSet.clear();

        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 0.1);

        //Drive and Intake
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, isAudience ? rowX + 4 : rowX, ySequence[1], AngleUnit.DEGREES, -90)), 0.5));
        intake.intakeGO(1);
        turret.setShooterVelocity(-1);
        intake.kickerGO(-0.5);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.0);
        actionSet.clear();


        //back up
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, isAudience ? rowX + 4 : rowX, ySequence[2], AngleUnit.DEGREES, -90))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 0.8);
        actionSet.clear();

        intake.intakeGO(0);
        turret.setShooterVelocity(0);
        intake.kickerGO(0);
    }

    public void humanPlayerIntake(){

        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 44, -63, AngleUnit.DEGREES, 0))));
        intake.intakeGO(1);
        intake.kickerGO(-0.5);
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3);
        actionSet.clear();

        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 64, -63, AngleUnit.DEGREES, -5)), 0.2));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.5);
        actionSet.clear();

        turret.setShooterVelocity(0);
        intake.intakeGO(0);
        intake.kickerGO(0);

    }


    public void intakeHumanPlayerPreset() {
        actionSet.clear();

        //drive
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 58, -42, AngleUnit.DEGREES, -90))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 2);
        actionSet.clear();
        intake.intakeGO(-1);
        intake.kickerGO(-1);
        turret.setShooterVelocityTicks(-120);

        //drive
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 58, -63, AngleUnit.DEGREES, -80))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.5);
        actionSet.clear();

        //drive
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 58, -52, AngleUnit.DEGREES, -80))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.5);
        actionSet.clear();

        //drive
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 61, -52, AngleUnit.DEGREES, -80))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.5);
        actionSet.clear();

        //back up
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 61, -63, AngleUnit.DEGREES, -80))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.5);
        actionSet.clear();
        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 0.5);
        intake.intakeGO(0);
        intake.kickerGO(0);
        turret.setShooterVelocityTicks(0);
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

    public void intakeFromGate() {
        actionSet.clear();

        //drive to gate opening pos midpoint
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 13, -48, AngleUnit.DEGREES, 225))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1.5);
        actionSet.clear();

        //drive to gate opening pos
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 12, -57, AngleUnit.DEGREES, 235))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 3.5);
        actionSet.clear();

        //drive forward
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 13, -60, AngleUnit.DEGREES, 235))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 2);
        actionSet.clear();

        //drive into gate
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 7.6, -62, AngleUnit.DEGREES, 235))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 0.1);
        actionSet.clear();
        intake.intakeGO(-1);
        intake.kickerGO(-1);
        turret.setShooterVelocityTicks(-120);

        //back up
        actionSet.add(() -> drivetrain.driveToPose(allianceFlip(isRed, new Pose2D(DistanceUnit.INCH, 15, -61.5, AngleUnit.DEGREES, 245))));
        AutoUtil.runActionsConcurrent(actionSet, periodicSet, TimeUnit.SECONDS, 1);
        actionSet.clear();
        AutoUtil.runTimedLoop(periodicSet, TimeUnit.SECONDS, 0.5);
        intake.intakeGO(0);
        intake.kickerGO(0);
        turret.setShooterVelocityTicks(0);
    }

    public void delay(TimeUnit timeUnit, double time){
        AutoUtil.runTimedLoop(periodicSet, timeUnit, time);
    }

    private Pose2D allianceFlip(boolean isRed, Pose2D pose){
        return PoseEstimator.allianceFlip(isRed, pose);
    }

}
