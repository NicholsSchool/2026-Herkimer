package org.firstinspires.ftc.teamcode.subsystems.turret;

import android.util.Size;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.function.IntSupplier;

public class TurretIOReal implements TurretIO, TurretConstants {

    //the servos that control the angle of our shot
    Servo rapidRedirector, rapidRedirector2;
    //the actual shooter wheels
    DcMotorEx artifactAccelerator, artifactAccelerator2;
    //the servos that turn our turret
    CRServo turretTurner1, turretTurner2;

    IntSupplier turretEncoderSupplier;

    Intake intake;

    DigitalChannel magnet;




    private AprilTagProcessor tagProcessor;

    public TurretIOReal(HardwareMap hwMap, Intake intake){

        this.intake = intake;
        artifactAccelerator = hwMap.get(DcMotorEx.class, "AA");
        artifactAccelerator2 = hwMap.get(DcMotorEx.class, "AA2");
        rapidRedirector = hwMap.get(Servo.class, "RR1");
        rapidRedirector2 = hwMap.get(Servo.class, "RR2");
        turretTurner1 = hwMap.get(CRServo.class, "TT1");
        turretTurner2 = hwMap.get(CRServo.class, "TT2");
        magnet = hwMap.get(DigitalChannel.class, "magnet");

        tagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        VisionPortal.Builder vBuilder = new VisionPortal.Builder();

        vBuilder.setCamera(hwMap.get(WebcamName.class, "W"));
        vBuilder.addProcessor(tagProcessor);
        vBuilder.setCameraResolution(new Size(frameWidth, 720));
        vBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        magnet.setMode(DigitalChannel.Mode.INPUT);

        VisionPortal vision = vBuilder.build();

        vision.resumeStreaming();
    }

    @Override
    public void updateInputs (TurretIO.TurretIOInputs inputs){


            List<AprilTagDetection> result = tagProcessor.getDetections();
            if (!result.isEmpty()) {
                for (AprilTagDetection tag : result) {
                    if (tag.id == TAGID) {
                        inputs.tagDistance = tag.ftcPose.range;
                        inputs.offset = tag.center.x;

                    }
                }
            }else{
                inputs.offset = (double)frameWidth / 2;
            }

            inputs.artifactAcceleratorVelocity = ( artifactAccelerator2.getVelocity() - artifactAccelerator.getVelocity() ) / 2;

            if (!magnet.getState()){
                intake.resetTurretEncoder();
            }
            inputs.turretPos = intake.getTurretPos();

            inputs.redirectorPos = rapidRedirector.getPosition();

            inputs.magnetState = magnet.getState();


    }

    @Override
    public void setPowerTurretTurner(double power){
        turretTurner1.setPower(power);
        turretTurner2.setPower(power);
    }

    @Override
    public void setPosRapidRedirector(double pos){
        rapidRedirector.setPosition(pos);
        rapidRedirector2.setPosition(pos);
    }


    @Override
    public void setVelocityArtifactAccelerator(double velocity){
        artifactAccelerator.setVelocity(-velocity);
        artifactAccelerator2.setVelocity(velocity);
    }

//    //true is false (when the turret is at the magnet, it returns false)
//    public boolean magnetValue(){
//        return magnet.getState();
//    }












}
