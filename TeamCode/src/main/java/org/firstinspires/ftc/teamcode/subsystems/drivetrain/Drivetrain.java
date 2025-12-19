package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.SubsystemBase;

public class Drivetrain extends SubsystemBase implements DrivetrainConstants {

    private DrivetrainIO io;
    private final DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
    public Pose2D setpoint;
    public PoseEstimator poseEstimator;
    public PIDController drivePID;
    public PIDController turnController;
    private Vector PIDDriveVector = new Vector(0, 0);
    public Pose2D initialPose;

    public Drivetrain(DrivetrainIO io, Pose2D initialPose, HardwareMap hwMap){
        this.io = io;
        this.initialPose = initialPose;
        poseEstimator = new PoseEstimator(hwMap, initialPose, false);
        drivePID = new PIDController(1.5,0,.3);
        turnController = new PIDController(1,0,.2);
        while (io.imuIsReady() != GoBildaPinpointDriver.DeviceStatus.READY) {io.updateInputs(inputs);}
        io.setPinpointPos(initialPose);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        poseEstimator.update();
    }

    public Pose2D getInitialPose(){
        return initialPose;
    }

    public void setPosition(){
        io.setPinpointPos(new Pose2D(DistanceUnit.METER , 2 ,2 ,AngleUnit.DEGREES, 90));
    }

    public void drive(double y, double x, double turn){
        io.setDriveMotorPower(y, x, turn);
    }

    public void driveField(double y, double x, double turn, double headingOffset){io.setFieldDriveMotorPower(y, x ,turn, headingOffset);}
    
    public void lightColor (double color){
        io.setBackLightColor(color);
    }

    public void eggPos(double pos1, double pos2) { io.setEggPos(pos1, pos2); }

    public void resetIMU(){io.resetIMU();}

    public double getIMU(){
        return inputs.imuHeading;
    }

    public double getPosX(){
        return inputs.posX;
    }

    public double getPosY(){
        return inputs.posY;
    }

    public AutoUtil.AutoActionState driveToPose(Pose2D targetPose){
        this.setpoint = targetPose;

        if( poseEstimator.getPose() == null )
            return AutoUtil.AutoActionState.RUNNING;

        Vector diffVector = new Vector(targetPose.getX(DistanceUnit.METER) - poseEstimator.getPose().getX(DistanceUnit.METER),
                targetPose.getY(DistanceUnit.METER) - poseEstimator.getPose().getY(DistanceUnit.METER));

        if ( diffVector.magnitude() < DRIVE_SETPOINT_THRESHOLD &&
                Math.abs(inputs.imuHeading - targetPose.getHeading(AngleUnit.DEGREES) ) < TURN_SETPOINT_THRESHOLD) {
            return AutoUtil.AutoActionState.FINISHED;
        }
        double driveMagnitude = drivePID.calculate(diffVector.magnitude(), 0);
        double driveAngle = diffVector.angle();

        PIDDriveVector = new Vector(

                driveMagnitude * Math.cos(driveAngle),
                driveMagnitude * Math.sin(driveAngle)
        );

        PIDDriveVector.clipMagnitude(1.0);

        double error = Angles.clipRadians(poseEstimator.getPose().getHeading(AngleUnit.RADIANS) - targetPose.getHeading(AngleUnit.RADIANS));

        io.setFieldDriveMotorPower(PIDDriveVector.y, PIDDriveVector.x, -turnController.calculate(error, 0), 90); //TODO: Change this if the robot does not go where it should

        return AutoUtil.AutoActionState.RUNNING;
    }

    public double getDriveMagnitude(){
        return PIDDriveVector.magnitude();
    }

    public void sendDashboardPacket(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setAlpha(0.7)
                .setStrokeWidth(1)
                .setStroke(poseEstimator.isUsingLL ? "Green" : "Red")
                .strokeCircle(
                        poseEstimator.getPose().getX(DistanceUnit.INCH),
                        poseEstimator.getPose().getY(DistanceUnit.INCH),
                        9
                )
                .setStroke("Green")
                .strokeLine(
                        poseEstimator.getPose().getX(DistanceUnit.INCH),
                        poseEstimator.getPose().getY(DistanceUnit.INCH),
                        poseEstimator.getPose().getX(DistanceUnit.INCH) + (9 * Math.cos(poseEstimator.getPose().getHeading(AngleUnit.RADIANS))),
                        poseEstimator.getPose().getY(DistanceUnit.INCH) + (9 * Math.sin(poseEstimator.getPose().getHeading(AngleUnit.RADIANS)))
                )
                .setStroke("Purple")
                .strokeCircle(setpoint.getX(DistanceUnit.INCH), setpoint.getY(DistanceUnit.INCH), DRIVE_SETPOINT_THRESHOLD)
                .setStroke("Cyan")
                .strokeLine(
                        poseEstimator.getPose().getX(DistanceUnit.INCH),
                        poseEstimator.getPose().getY(DistanceUnit.INCH),
                        poseEstimator.getPose().getX(DistanceUnit.INCH) + PIDDriveVector.x,
                        poseEstimator.getPose().getY(DistanceUnit.INCH) + PIDDriveVector.y
                );
        dashboard.sendTelemetryPacket(packet);
    }


}


