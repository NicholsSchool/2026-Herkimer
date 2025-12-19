package org.firstinspires.ftc.teamcode.subsystems.turret;

import android.graphics.Bitmap;
import android.graphics.Camera;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.IntSupplier;

public class TurretIOReal implements TurretIO, TurretConstants {

    //the servos that control the angle of our shot
    Servo rapidRedirector, rapidRedirector2;
    //the actual shooter wheel (one motor on both sides attached to the same shaft)
    DcMotorEx artifactAccelerator, artifactAccelerator2, turretEncoder;
    //the servos that turn our turret
    CRServo turretTurner1, turretTurner2;



    Intake intake;
    //the magnet sensor that acts as a limit switch for our turret
    DigitalChannel magnet;



    public TurretIOReal(HardwareMap hwMap){

        artifactAccelerator = hwMap.get(DcMotorEx.class, "AA");
        artifactAccelerator2 = hwMap.get(DcMotorEx.class, "AA2");
        rapidRedirector = hwMap.get(Servo.class, "RR1");
        rapidRedirector2 = hwMap.get(Servo.class, "RR2");
        turretTurner1 = hwMap.get(CRServo.class, "TT1");
        turretTurner2 = hwMap.get(CRServo.class, "TT2");
        turretEncoder = hwMap.get(DcMotorEx.class, "intake");
        magnet = hwMap.get(DigitalChannel.class, "magnet");

        magnet.setMode(DigitalChannel.Mode.INPUT);

    }

    @Override
    public void updateInputs (TurretIO.TurretIOInputs inputs){
        inputs.turretAngle = (turretEncoder.getCurrentPosition() * 0.00009) + Math.PI / 2;
        inputs.magnetState = magnet.getState();
        inputs.rawTurretAngle = turretEncoder.getCurrentPosition();
        inputs.shooterVelocity = artifactAccelerator.getVelocity();

        if (!inputs.magnetState) {
            turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void shooterSetVelocity(double velocity){
        artifactAccelerator.setVelocity(velocity);
        artifactAccelerator2.setVelocity(-velocity);
    }

    @Override
    public void redirectorSetPosition(double pos){
        rapidRedirector.setPosition(pos);
        rapidRedirector2.setPosition(pos);
    }

    @Override
    public void turretSetPower(double power){
        turretTurner2.setPower(power);
        turretTurner1.setPower(power);

    }














}
