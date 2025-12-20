package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.Angles;
import org.firstinspires.ftc.teamcode.math_utils.AutoUtil;
import org.firstinspires.ftc.teamcode.math_utils.MotionProfile;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;

public class DrivetrainIOReal implements DrivetrainIO, DrivetrainConstants {

    DcMotorEx backRight, backLeft, frontRight, frontLeft;
    //    Limelight3A LL;
    //    Servo tLight, mLight, bLight;
    Servo regg, legg;

    public Pose2D setpoint;

    public DrivetrainIOReal(HardwareMap hwMap) {

        frontLeft = hwMap.get(DcMotorEx.class, "fL");
        frontRight = hwMap.get(DcMotorEx.class, "fR");
        backLeft = hwMap.get(DcMotorEx.class, "bL");
        backRight = hwMap.get(DcMotorEx.class, "bR");
        legg = hwMap.get(Servo.class, "legg");
        regg = hwMap.get(Servo.class, "regg");

        setpoint = new Pose2D(DistanceUnit.METER, 0.0, 0.0, AngleUnit.RADIANS, 0.0);
//
//        tLight = hwMap.get(Servo.class, "tLight");
//        mLight = hwMap.get(Servo.class, "mLight");
//        bLight = hwMap.get(Servo.class, "bLight");

        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void updateInputs(DrivetrainIO.DrivetrainIOInputs inputs) {

        inputs.imuHeading = PoseEstimator.getPose().getHeading(AngleUnit.DEGREES);

    }

    @Override
    public void setDriveMotorPower(double y, double x, double turn) {

        backRight.setPower(y - x + turn);
        backLeft.setPower(y + x - turn);
        frontRight.setPower(y + x + turn);
        frontLeft.setPower(y - x - turn);

    }

    @Override
    public void setFieldDriveMotorPower(double y, double x, double turn, double headingOffset) {
        double offset = PoseEstimator.getPose().getHeading(AngleUnit.RADIANS) + headingOffset;
        double fieldX = x * Math.cos(offset) - y * Math.sin(offset);
        double fieldY = x * Math.sin(offset) + y * Math.cos(offset);

        setDriveMotorPower(fieldY, fieldX, turn);
    }

    ;

    @Override
    public void setEggPos(double pos1, double pos2) {
        regg.setPosition(pos1);
        legg.setPosition(pos2);
    }

}
