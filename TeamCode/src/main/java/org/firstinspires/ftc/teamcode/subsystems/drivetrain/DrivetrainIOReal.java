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
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;

public class DrivetrainIOReal implements DrivetrainIO, DrivetrainConstants {

    DcMotorEx backRight, backLeft, frontRight, frontLeft;
//    Limelight3A LL;
    GoBildaPinpointDriver imu;
//    Servo tLight, mLight, bLight;
    Servo regg, legg;


    public DrivetrainIOReal(HardwareMap hwMap){

        frontLeft = hwMap.get(DcMotorEx.class, "fL");
        frontRight = hwMap.get(DcMotorEx.class, "fR");
        backLeft = hwMap.get(DcMotorEx.class, "bL");
        backRight = hwMap.get(DcMotorEx.class, "bR");
        legg = hwMap.get(Servo.class, "legg");
        regg = hwMap.get(Servo.class, "regg");


//        LL = hwMap.get(Limelight3A.class, "LL");

        imu = hwMap.get(GoBildaPinpointDriver.class, "imu");
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

        imu.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        imu.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

    }

    @Override
    public void updateInputs (DrivetrainIO.DrivetrainIOInputs inputs){

        inputs.imuReading = imu.getHeading(AngleUnit.DEGREES);

    }

    @Override
    public void setDriveMotorPower (double y, double x, double turn){

        backRight.setPower(y - x + turn);
        backLeft.setPower(y + x - turn);
        frontRight.setPower(y + x + turn);
        frontLeft.setPower(y - x - turn);

    }

    @Override
    public void setFieldDriveMotorPower (double y, double x, double turn, double headingOffset){
        double offset = imu.getHeading(AngleUnit.RADIANS) + headingOffset;
        double fieldX = x * Math.cos(offset) - y * Math.sin(offset);
        double fieldY = x * Math.sin(offset) + y * Math.cos(offset);

        setDriveMotorPower(fieldY, fieldX, turn);

    };

    @Override
    public void setEggPos (double pos1, double pos2){
        regg.setPosition(pos1);
        legg.setPosition(pos2);
    }

    @Override
    public void resetIMU (){
        imu.resetPosAndIMU();
    }

    @Override
    public void updateIMU (){
        imu.update();
    }





}
