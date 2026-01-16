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
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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

    //the actual shooter wheel (one motor on both sides attached to the same shaft)
    DcMotorEx artifactAccelerator, turretEncoder;
    //the top wheel that "redirects" the artifact
    DcMotorEx rapidRedirector;
    //the servos that turn our turret
    CRServo turretTurner1, turretTurner2;
    //the magnet sensor that acts as a limit switch for our turret
    DigitalChannel magnet;

    public boolean isRed;


    public TurretIOReal(HardwareMap hwMap, boolean isRed){

        artifactAccelerator = hwMap.get(DcMotorEx.class, "shooter");
        rapidRedirector = hwMap.get(DcMotorEx.class, "redirector");
        turretTurner1 = hwMap.get(CRServo.class, "TT1");
        turretTurner2 = hwMap.get(CRServo.class, "TT2");
        turretEncoder = hwMap.get(DcMotorEx.class, "intake");
        magnet = hwMap.get(DigitalChannel.class, "magnet");

        artifactAccelerator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rapidRedirector.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rapidRedirector.setVelocityPIDFCoefficients(120,7,0.0,0.0);
        artifactAccelerator.setVelocityPIDFCoefficients(320,20,0.0,50);


        this.isRed = isRed;
        magnet.setMode(DigitalChannel.Mode.INPUT);
//        List<AprilTagDetection> result = aprilTag.getDetections();
//        if(!result.isEmpty()){
//            for(AprilTagDetection tag: result){
//                if(tag.id == TAGID){
//                    inputs.tagDistance = tag.ftcPose.range;
//                    inputs.tagX = tag.center.x;
//                }
//            }
//        }else{
//            inputs.tagX = (double)frameWidth / 2;
//        }



    }

    @Override
    public void updateInputs (TurretIO.TurretIOInputs inputs){
        inputs.turretAngle = (turretEncoder.getCurrentPosition() / 7274.78146);
        inputs.magnetState = magnet.getState();
        inputs.rawTurretAngle = turretEncoder.getCurrentPosition();
        inputs.shooterVelocity = artifactAccelerator.getVelocity();
        inputs.redirectorVelocity = rapidRedirector.getVelocity();
        inputs.redirectorPower = rapidRedirector.getPower();

        inputs.aprilTagPos = (isRed ? redTagPos : blueTagPos);
    }

    @Override
    public void shooterSetVelocity(double velocity){
        artifactAccelerator.setVelocity(velocity);
    }

    @Override
    public void redirectorSetVelocity(double velocity){
        rapidRedirector.setVelocity(velocity);
    }

    @Override
    public void turretSetPower(double power){
        turretTurner2.setPower(power);
        turretTurner1.setPower(power);

    }

    @Override
    public void resetTurretEncoder(TurretIO.TurretIOInputs inputs){
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }














}
