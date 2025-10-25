package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;

public class DrivetrainIOReal implements DrivetrainIO, DrivetrainConstants {

    DcMotorEx backRight, backLeft, frontRight, frontLeft;
    Limelight3A LL;
    GoBildaPinpointDriver imu;
    Servo tLight, mLight, bLight;


    public DrivetrainIOReal(HardwareMap hwMap){

        frontLeft = hwMap.get(DcMotorEx.class, "fL");
        frontRight = hwMap.get(DcMotorEx.class, "fR");
        backLeft = hwMap.get(DcMotorEx.class, "bL");
        backRight = hwMap.get(DcMotorEx.class, "bR");

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setVelocityPIDFCoefficients(fL_P, fL_I, fL_D, 0.0);
        frontRight.setVelocityPIDFCoefficients(fR_P, fR_I, fR_D, 0.0);
        backLeft.setVelocityPIDFCoefficients(bL_P, bL_I, bL_D, 0.0);
        backRight.setVelocityPIDFCoefficients(bR_P, bR_I, bR_D, 0.0);

        LL = hwMap.get(Limelight3A.class, "LL");

        imu = hwMap.get(GoBildaPinpointDriver.class, "imu");

        tLight = hwMap.get(Servo.class, "tLight");
        mLight = hwMap.get(Servo.class, "mLight");
        bLight = hwMap.get(Servo.class, "bLight");

    }

    @Override
    public void updateInputs (DrivetrainIO.DrivetrainIOInputs inputs){

        inputs.imuReading = imu.getHeading(AngleUnit.DEGREES);

    }

    @Override
    public void setDriveMotorPower (double y, double x, double turn){

        backRight.setVelocity(y + x - turn);
        backLeft.setVelocity(y - x + turn);
        frontRight.setVelocity(y - x - turn);
        frontLeft.setVelocity(y + x + turn);

    }

    @Override
    public void setFieldDriveMotorPower (double y, double x, double turn, double headingOffset){
        double offset = imu.getHeading(AngleUnit.RADIANS) + headingOffset;
        double fieldX = x * Math.cos(-offset) - y * Math.sin(-offset);
        double fieldY = x * Math.sin(-offset) + y * Math.cos(-offset);

        setDriveMotorPower(fieldY, fieldX, turn);
    };



}
