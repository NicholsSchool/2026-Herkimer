package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeIO;

public class DrivetrainIOReal implements DrivetrainIO, DrivetrainConstants {

    DcMotorEx backRight, backLeft, frontRight, frontLeft;
    Limelight3A LL;
    GoBildaPinpointDriver imu;


    public DrivetrainIOReal(HardwareMap hwMap){

        backRight = hwMap.get(DcMotorEx.class, "bR");
        backLeft = hwMap.get(DcMotorEx.class, "bL");
        frontRight = hwMap.get(DcMotorEx.class, "fR");
        frontLeft = hwMap.get(DcMotorEx.class, "fL");

        LL = hwMap.get(Limelight3A.class, "LL");

        imu = hwMap.get(GoBildaPinpointDriver.class, "imu");

    }

   //:3


    @Override
    public void updateInputs (DrivetrainIO.DrivetrainIOInputs inputs){

    }

}
