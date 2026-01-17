package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

@Config
public class Drivetrain extends SubsystemBase implements DrivetrainConstants {

    private DrivetrainIO io;
    private final DrivetrainIO.DrivetrainIOInputs inputs = new DrivetrainIO.DrivetrainIOInputs();
    public Pose2D setpoint = new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0);
    public PIDController drivePIDX, drivePIDY;
    public static double kDP = 8, kDI = 1.8, kDD = 1.7; // "konstant Drivetrain P/I/D - shouldn't be different since they're field X and Y
    public PIDController turnController;
    private Vector drivePIDError = new Vector(0, 0);
    private double turnPIDError = 0.0;
    private Vector PIDDriveVector = new Vector(0, 0);

    public Drivetrain(DrivetrainIO io, HardwareMap hwMap){
        this.io = io;
        drivePIDX = new PIDController(kDP, kDI, kDD);
        drivePIDX.setIZone(0.5);
        drivePIDY = new PIDController(kDP, kDI, kDD);
        drivePIDY.setIZone(0.5);
        turnController = new PIDController(2.3, 0.04, 0.2);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public void drive(double y, double x, double turn){
        io.setDriveMotorPower(y, x, turn);
    }

    public void driveField(double y, double x, double turn, double headingOffset){io.setFieldDriveMotorPower(y, x ,turn, headingOffset);}

    public void eggPos(double pos1, double pos2) { io.setEggPos(pos1, pos2); }

    public double getIMU(){
        return inputs.imuHeading;
    }

    public Pose2D getPose() { return PoseEstimator.getPose(); }

//    public void resetPID() { drivePID.reset(); turnController.reset(); }

    public AutoUtil.AutoActionState driveToPose(Pose2D targetPose) {
        return driveToPose(targetPose, AUTO_BASE_SPEED);
    }

    public AutoUtil.AutoActionState driveToPose(Pose2D targetPose, double speed){
        this.setpoint = targetPose;

        if( PoseEstimator.getPose() == null )
            return AutoUtil.AutoActionState.RUNNING;

        Vector diffVector = new Vector(targetPose.getX(DistanceUnit.METER) - PoseEstimator.getPose().getX(DistanceUnit.METER),
                targetPose.getY(DistanceUnit.METER) - PoseEstimator.getPose().getY(DistanceUnit.METER));

        if ( Math.abs(diffVector.magnitude()) < DRIVE_POSITION_THRESHOLD &&
                Math.abs(Angles.clipDegrees(inputs.imuHeading - targetPose.getHeading(AngleUnit.DEGREES))) < TURN_POSITION_THRESHOLD) {
            drive(0, 0, 0);
            return AutoUtil.AutoActionState.FINISHED;
        }
        double driveMagnitudeX = drivePIDX.calculate(diffVector.x, 0);
        double driveMagnitudeY = drivePIDY.calculate(diffVector.y, 0);

        drivePIDError = diffVector;
        turnPIDError = Angles.clipDegrees(inputs.imuHeading - targetPose.getHeading(AngleUnit.DEGREES));

        PIDDriveVector = new Vector(

                driveMagnitudeX * speed,
                driveMagnitudeY * speed
        );

        PIDDriveVector.clipMagnitude(1.0);

        double error = Angles.clipRadians(PoseEstimator.getPose().getHeading(AngleUnit.RADIANS) - targetPose.getHeading(AngleUnit.RADIANS));

        io.setFieldDriveMotorPower(-PIDDriveVector.y, PIDDriveVector.x, -turnController.calculate(error, 0), 90);

        return AutoUtil.AutoActionState.RUNNING;
    }

    public Vector getDrivePIDError(){
        return drivePIDError;
    }

    public double getTurnPIDError(){
        return turnPIDError;
    }

    public void sendDashboardPacket(FtcDashboard dashboard) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setAlpha(0.7)
                .setStrokeWidth(1)
                .setStroke("Green")
                .strokeCircle(
                        PoseEstimator.getPose().getX(DistanceUnit.INCH),
                        PoseEstimator.getPose().getY(DistanceUnit.INCH),
                        9
                )
                .setStroke("Green")
                .strokeLine(
                        PoseEstimator.getPose().getX(DistanceUnit.INCH),
                        PoseEstimator.getPose().getY(DistanceUnit.INCH),
                        PoseEstimator.getPose().getX(DistanceUnit.INCH) + (9 * Math.cos(PoseEstimator.getPose().getHeading(AngleUnit.RADIANS))),
                        PoseEstimator.getPose().getY(DistanceUnit.INCH) + (9 * Math.sin(PoseEstimator.getPose().getHeading(AngleUnit.RADIANS)))
                )
                .setStroke("Purple")
                .strokeCircle(setpoint.getX(DistanceUnit.INCH), setpoint.getY(DistanceUnit.INCH), DistanceUnit.INCH.fromMeters(DRIVE_POSITION_THRESHOLD))
                .setStroke("Cyan")
                .strokeLine(
                        PoseEstimator.getPose().getX(DistanceUnit.INCH),
                        PoseEstimator.getPose().getY(DistanceUnit.INCH),
                        PoseEstimator.getPose().getX(DistanceUnit.INCH) + PIDDriveVector.x,
                        PoseEstimator.getPose().getY(DistanceUnit.INCH) + PIDDriveVector.y
                );
        dashboard.sendTelemetryPacket(packet);
    }


}


