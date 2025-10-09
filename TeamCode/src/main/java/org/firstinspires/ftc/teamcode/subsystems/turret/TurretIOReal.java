package org.firstinspires.ftc.teamcode.subsystems.turret;

import android.util.Size;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class TurretIOReal implements TurretIO, TurretConstants {

    Servo rapidRedirector;
    DcMotorEx artifactAccelerator;
    CRServo turretTurner1, turretTurner2;
    HardwareMap hwMap;



    private AprilTagProcessor tagProcessor;

    public TurretIOReal(){

        artifactAccelerator = hwMap.get(DcMotorEx.class, "AA");
        rapidRedirector = hwMap.get(Servo.class, "RR");
        turretTurner1 = hwMap.get(CRServo.class, "TT1");
        turretTurner2 = hwMap.get(CRServo.class, "TT2");

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal.Builder vBuilder = new VisionPortal.Builder();

        vBuilder.setCamera(hwMap.get(WebcamName.class, "webcam"));
        vBuilder.addProcessor(tagProcessor);
        vBuilder.setCameraResolution(new Size(frameWidth, 720));
        vBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        VisionPortal vision = vBuilder.build();

        vision.resumeStreaming();
    }

    @Override
    public void updateInputs (TurretIO.TurretIOInputs inputs){

            List<AprilTagDetection> result = tagProcessor.getDetections();
            if (!result.isEmpty()) {
                for (AprilTagDetection tag : result) {
                    if (tag.id == TAGID) {
                        inputs.tag = tag;
                    }
                }
            }

            inputs.getArtifactAcceleratorVelocity = new double[] {artifactAccelerator.getVelocity(), 0.0};


    }
}
