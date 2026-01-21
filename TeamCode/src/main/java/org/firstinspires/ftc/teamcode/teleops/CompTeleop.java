package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

import java.util.logging.Logger;

@TeleOp (name = "comptele")
public class CompTeleop extends OpMode {


    public Drivetrain drivetrain;
    public Turret turret;
    public Intake intake;
    private FtcDashboard dashboard;
    private boolean isRed = false;

    @Override
    public void init(){
        LightManager.inititalize(hardwareMap);
        PoseEstimator.init(hardwareMap, new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0), false, false);
        drivetrain = new Drivetrain(new DrivetrainIOReal(hardwareMap), hardwareMap);
        intake = new Intake(new IntakeIOReal(hardwareMap));
        turret = new Turret(new TurretIOReal(hardwareMap, isRed));
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        turret.resetTurretEncoder();
    }
    @Override
    public void init_loop(){
        if(gamepad2.aWasPressed()){
            isRed = !isRed;
        }
        telemetry.addLine("[G2 A] Teleop Alliance Color: " + (isRed ? "RED" : "BLUE"));
        telemetry.update();
    }

    @Override
    public void start() {
        if (isRed) {
            turret.setTagID(24);
            LightManager.GoBildaLights.setLights(new double[] {LightManager.LightConstants.Red, LightManager.LightConstants.Red, LightManager.LightConstants.Red});
        } else {
            turret.setTagID(20);
            LightManager.GoBildaLights.setLights(new double[] {LightManager.LightConstants.Blue, LightManager.LightConstants.Blue, LightManager.LightConstants.Blue});
        }

    }

    @Override
    public void loop(){

        //update all the subsystems
        turret.periodic();
        drivetrain.periodic();
        intake.periodic();
        PoseEstimator.periodic();

        //field-oriented driving on controller1
        drivetrain.driveField(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, isRed ? -Math.PI / 2 : Math.PI / 2);

        //kickstand/climb on controller1
        if(gamepad1.x){
            drivetrain.eggPos(0,0);
        }else if(gamepad1.y){
            drivetrain.eggPos(1,1);
        }

        //reset the IMU to reset Field oriented on controller1
        if (gamepad1.dpad_up){
            PoseEstimator.resetIMU();
        }

        if (gamepad1.dpad_down){
            PoseEstimator.resetPoseToAutoStart(isRed);
        }

            //Compact on controller2
        if (gamepad2.y){
            intake.intakeGO(.8);
            intake.kickerGO(.8);
            turret.setShooterVelocity(0);
            turret.redirectorSetVelocity(0);
        }else if(gamepad2.b){
            //intake on controller2
            intake.intakeGO(.7);
            turret.setShooterVelocity(1);
            turret.redirectorSetVelocity(0);
            intake.kickerGO(-.7);
        }else if(gamepad2.a){
            //outtake on controller2
            intake.intakeGO(-0.5);
            intake.kickerGO(0.5);
            turret.setShooterVelocity(0);
            turret.redirectorSetVelocity(0);
        }else if (gamepad2.right_trigger > 0.2) {
            turret.autoAccelerate();
            LightManager.LEDStrip.setRPMLights(-turret.getShooterVelocity(), -turret.getAcceleratorSetpoint());
            if(turret.getShooterVelocity() <= turret.getAcceleratorSetpoint() || Math.abs(turret.getShooterVelocity() - turret.getAcceleratorSetpoint()) < TurretConstants.SHOOT_SPEED_TOLERANCE){
                intake.kickerGO(-0.9);
                intake.intakeGO(1);
            } else {
                intake.kickerGO(0);
                intake.intakeGO(0);
            }
        }else{
            //everything off
            turret.setShooterVelocity(0);
            turret.redirectorSetVelocity(0);
            intake.intakeGO(0);
            intake.kickerGO(0);
            LightManager.LEDStrip.clear();
        }

        //manual turret control
//        turret.turretSetPower(gamepad2.left_stick_x);

//        //shoot on controller2
//        if (gamepad2.right_trigger > 0.5) {
//            turret.runShooterForDistance();
//        }else{
//            turret.setShooterVelocity(0);
//        }

//        if (gamepad2.left_trigger > 0.2) {
//            turret.autoAim();
//            Logger.getLogger("CompTeleop Turret").info("Updated PID");
//        } else {
//            turret.turretSetPower(0);
//        }

        telemetry.addData("Turret position error", turret.getAimError(AngleUnit.DEGREES));
        telemetry.addData("Turret Position", turret.getTurretPosition(AngleUnit.DEGREES));
        telemetry.addData("setpoint", turret.getTurretSetpoint(AngleUnit.DEGREES));
        telemetry.addData("Distance to tag vector angle", Math.toDegrees(turret.aimTagDistance.angle()));
        telemetry.addData("robot heading", PoseEstimator.getPose().getHeading(AngleUnit.DEGREES));
        telemetry.addData("Distance from tag", turret.getTagDistance(DistanceUnit.METER));


        //FTC Dashboard telemetry packet
        drivetrain.sendDashboardPacket(dashboard);

    }
}
