package org.firstinspires.ftc.teamcode.subsystems.turret;

import android.graphics.Bitmap;
import android.graphics.Camera;
import android.graphics.Canvas;
import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
//import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.math_utils.PIDController;
import org.firstinspires.ftc.teamcode.math_utils.PIDFController;
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
@Config
public class TurretIOReal implements TurretIO, TurretConstants {

    //the actual shooter wheel (one motor on both sides attached to the same shaft)
    DcMotorEx artifactAccelerator1;
    DcMotorEx artifactAccelerator2;
    //The encoder for our turret, it's signal wire is in the kicker motor's port
    DcMotorEx turretEncoder;
    //the servos that turn our turret
    CRServo turretTurner1, turretTurner2;
    //the magnet sensor that acts as a limit switch for our turret
    DigitalChannel magnet;
    //mechanical stop
    Servo mechStop;
    //Hood angle servo
    Servo hood;

    Servo middleLight;
    public static double kVP = 400.0, kVI = 60.0, kVD = 30.0, kVF = 50.0;

    public TurretIOReal(HardwareMap hwMap){

        artifactAccelerator1 = hwMap.get(DcMotorEx.class, "Shooter1"); //left shooter
        artifactAccelerator2 = hwMap.get(DcMotorEx.class, "Shooter2"); //right shooter

        turretTurner1 = hwMap.get(CRServo.class, "TT1"); //right turret
        turretTurner2 = hwMap.get(CRServo.class, "TT2"); //left turret
        turretEncoder = hwMap.get(DcMotorEx.class, "kicker");
        magnet = hwMap.get(DigitalChannel.class, "magnet");
        mechStop = hwMap.get(Servo.class, "mechStop");
        hood = hwMap.get(Servo.class, "hood");
        middleLight = hwMap.get(Servo.class, "middleLight");

        artifactAccelerator1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        artifactAccelerator2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        artifactAccelerator1.setVelocityPIDFCoefficients(kVP,kVI,kVD,kVF);

        magnet.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void updateInputs (TurretIO.TurretIOInputs inputs){
        inputs.turretAngle = (turretEncoder.getCurrentPosition() / 7830.42222);
        inputs.magnetState = magnet.getState();
        inputs.rawTurretAngle = turretEncoder.getCurrentPosition();
        inputs.shooterVelocity = artifactAccelerator1.getVelocity();
        inputs.hoodAngle = hood.getPosition();
        inputs.turretPower = turretTurner1.getPower();
    }

    /**
     * Sets the velocity of the Shooter
     * @param velocity The desired velocity
     * */

    @Override
    public void shooterSetVelocity(double velocity){
        artifactAccelerator1.setVelocity(velocity);
        artifactAccelerator2.setVelocity(-velocity);
    }

    /**
     * Sets the color of the lights based on the Servo position given
     * (the lights are a Servo)
     * @param position The position of the Servo
     * */

    @Override
    public void setLightPosition(double position){
        middleLight.setPosition(position);
    }

    /**
     * Sets the position of the Hood
     * @param position The position to set the Hood to
     * */

    @Override
    public void hoodSetPosition(double position){
        hood.setPosition(position);
    }

    /**
     * Sets power to the Turret Servos
     * @param power The power given to the Servos
     * */

    @Override
    public void turretSetPower(double power){
        turretTurner2.setPower(-power);
        turretTurner1.setPower(-power);

    }

    /**
     * Sets the position of the Mechanical Stop
     * @param pos The desired position of the Mechanical Stop
     * */

    @Override
    public void setMechStopPosition(double pos){
        mechStop.setPosition(pos);
    }


    /**
     * Resets the Turret Encoder
     * @param inputs Supply the TurretIO.TurretIOInputs here
     * */

    @Override
    public void resetTurretEncoder(TurretIO.TurretIOInputs inputs){
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }














}
