package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Swerve{
    Module frontLeft, frontRight, backLeft, backRight;

    IMU imu;

    public Swerve(HardwareMap hwMap){
        frontLeft = new Module(hwMap, 0, "FLM", "FLS", "FLE", 1);
        frontRight = new Module(hwMap, 0, "FRM", "FRS", "FRE", 1);
        backRight = new Module(hwMap, 0, "BRM", "BRS", "BRE", 1);
        backLeft = new Module(hwMap, 0, "BLM", "BLS", "BLE", 1);

        imu = hwMap.get(IMU.class, "imu");

    }

    public double getHeading(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public void drive(double power, double angle, double turn, boolean x){
        if(x){
            frontLeft.driveModule(0.0, -Math.PI / 4);
            frontRight.driveModule(0.0,  Math.PI / 4);
            backLeft.driveModule(0.0,  Math.PI / 4);
            backRight.driveModule(0.0, -Math.PI / 4);
        }else {
            frontLeft.driveModule(power, angle + turn * Math.PI / 4);
            frontRight.driveModule(power, angle - turn * Math.PI / 4);
            backLeft.driveModule(power, angle - turn * Math.PI / 4);
            backRight.driveModule(power, angle + turn * Math.PI / 4);
        }
    }


}