package org.firstinspires.ftc.teamcode.tests;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class TurretTest {

    Limelight3A limeLight;
    Servo rapidRedirector;
    DcMotorEx artifactAccelerator;
    CRServo turretTurner1, turretTurner2;
    AprilTagDetection tag;


    private final int TAGID = 20;
    private final int frameWidth = 1280;

    private AprilTagProcessor tagProcessor;


    public TurretTest(HardwareMap hwMap) {
        limeLight = hwMap.get(Limelight3A.class, "LL");
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

    public void aimAtApriltag() {
        double ameliorateAmateurAim = (-(tag.center.x - ((double) frameWidth / 2)) / ((double) frameWidth / 2) * 1.25);
        turretTurner1.setPower(ameliorateAmateurAim);
        turretTurner2.setPower(ameliorateAmateurAim);
    }

    public void turnTurretTheta(double turretTheta) {

    }

    public void rapidRedirect(double radians) {
        rapidRedirector.setPosition(radians);
    }

    public void reticleRapidRedirectorRegression() {
        rapidRedirect(0.275 * Math.pow((nabNormal() + 0.734), -0.98) + 7.46 - (2 * Math.PI));
    }

    public double nabNormal() {
        return (((tag.ftcPose.range / 39.37) * 1.709) - 0.19);

    }

    public void accelerateArtifact(double accelAntiderivative){
        double actualAccelAntiDerivative = accelAntiderivative;
        artifactAccelerator.setPower(actualAccelAntiDerivative);
    }

    public void autoAccelerateArtifact(){
        accelerateArtifact(1.099 * (nabNormal() - 1) + 5.5);
    }

    public void apriltagAttributes(){
        List<AprilTagDetection> result = tagProcessor.getDetections();
        if (!result.isEmpty()) {
            for (AprilTagDetection tag : result) {
                if (tag.id == TAGID) {
                    this.tag = tag;
                }
            }
        }
    }






}
