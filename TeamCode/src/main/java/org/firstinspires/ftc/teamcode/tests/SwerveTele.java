package org.firstinspires.ftc.teamcode.tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "SwerveTele")
public class SwerveTele extends OpMode {

    Swerve swerve;

    @Override
    public void init() {

        swerve = new Swerve(hardwareMap);

    }

    @Override
    public void loop() {
        double power = Math.hypot(gamepad1.left_stick_y, gamepad1.left_stick_x);
        double angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x);

        swerve.drive(power, angle - swerve.getHeading(), gamepad1.right_stick_x, gamepad1.x);



    }
}
